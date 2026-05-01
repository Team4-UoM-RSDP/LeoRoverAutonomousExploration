#!/usr/bin/env python3
"""
Leo Rover Frontier-Based Autonomous Exploration Node
ROS2 Jazzy  |  v3.2  —  Wavefront Frontier Detection (WFD)

v3.2 changes:
  - Converts LaserScan points into the robot base frame before safety checks.
  - Supports real lidar extrinsics (front-mounted, yaw-offset lidar).
  - Uses rectangular body-clearance checks instead of a large circular bubble.
  - Keeps no-spin avoidance: back-up + gentle curve only.
  - Compatible with both simulation and real hardware through parameters.
"""

import math
import subprocess
from collections import deque
from enum import Enum, auto
from typing import Deque, List, Optional, Tuple

import numpy as np

import rclpy
import rclpy.time
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap

from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

# ─── WFD constants ──────────────────────────────────────────────────────────
OCC_THRESHOLD = 10          # occupancy grid cost above which a cell is occupied
MIN_FRONTIER_SIZE = 5       # minimum cluster size to consider as a frontier

# Costmap threshold: cells with cost >= this are in inflation / lethal zone
COSTMAP_LETHAL_THRESH = 70  # range 0-100 (nav2 scale)


# =============================================================================
#  WFD data structures (ported from nav2_wavefront_frontier_exploration)
# =============================================================================

class OccupancyGrid2d:
    """Lightweight wrapper around nav_msgs/OccupancyGrid for WFD."""

    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 100
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, grid_msg: OccupancyGrid) -> None:
        self.map = grid_msg

    def getCost(self, mx: int, my: int) -> int:
        return self.map.data[self._getIndex(mx, my)]

    def getSize(self) -> Tuple[int, int]:
        return (self.map.info.width, self.map.info.height)

    def getSizeX(self) -> int:
        return self.map.info.width

    def getSizeY(self) -> int:
        return self.map.info.height

    def getResolution(self) -> float:
        return self.map.info.resolution

    def mapToWorld(self, mx: int, my: int) -> Tuple[float, float]:
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution
        return (wx, wy)

    def worldToMap(self, wx: float, wy: float) -> Tuple[int, int]:
        if (wx < self.map.info.origin.position.x or
                wy < self.map.info.origin.position.y):
            raise Exception("World coordinates out of bounds")

        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)

        if my >= self.map.info.height or mx >= self.map.info.width:
            raise Exception("Out of bounds")

        return (mx, my)

    def _getIndex(self, mx: int, my: int) -> int:
        return my * self.map.info.width + mx


class FrontierPoint:
    __slots__ = ("classification", "mapX", "mapY")

    def __init__(self, x: int, y: int) -> None:
        self.classification = 0
        self.mapX = x
        self.mapY = y


class FrontierCache:
    """Hash-based cache avoiding duplicate FrontierPoint objects."""

    def __init__(self) -> None:
        self.cache: dict = {}

    def getPoint(self, x: int, y: int) -> FrontierPoint:
        idx = self._cantorHash(x, y)
        if idx in self.cache:
            return self.cache[idx]
        pt = FrontierPoint(x, y)
        self.cache[idx] = pt
        return pt

    def _cantorHash(self, x: int, y: int) -> int:
        return ((x + y) * (x + y + 1) // 2) + y

    def clear(self) -> None:
        self.cache = {}


class PointClassification(Enum):
    MapOpen = 1
    MapClosed = 2
    FrontierOpen = 4
    FrontierClosed = 8


# =============================================================================
#  WFD utility functions
# =============================================================================

def _centroid(arr: list) -> Tuple[float, float]:
    """Compute the centroid of a list of (x, y) tuples."""
    a = np.array(arr)
    return float(np.mean(a[:, 0])), float(np.mean(a[:, 1]))


def _getNeighbors(
    point: FrontierPoint,
    costmap: OccupancyGrid2d,
    fCache: FrontierCache,
) -> List[FrontierPoint]:
    """8-connected neighbors within map bounds (excluding the point itself)."""
    neighbors = []
    for x in range(point.mapX - 1, point.mapX + 2):
        for y in range(point.mapY - 1, point.mapY + 2):
            if x == point.mapX and y == point.mapY:
                continue
            if 0 <= x < costmap.getSizeX() and 0 <= y < costmap.getSizeY():
                neighbors.append(fCache.getPoint(x, y))
    return neighbors


def _isFrontierPoint(
    point: FrontierPoint,
    costmap: OccupancyGrid2d,
    fCache: FrontierCache,
) -> bool:
    """
    A frontier point is an unknown cell adjacent to at least one free cell,
    with no high-cost (occupied) neighbors.
    """
    if costmap.getCost(point.mapX, point.mapY) != OccupancyGrid2d.CostValues.NoInformation.value:
        return False

    hasFree = False
    for n in _getNeighbors(point, costmap, fCache):
        cost = costmap.getCost(n.mapX, n.mapY)
        if cost > OCC_THRESHOLD:
            return False
        if cost == OccupancyGrid2d.CostValues.FreeSpace.value:
            hasFree = True

    return hasFree


def _findFree(
    mx: int, my: int,
    costmap: OccupancyGrid2d,
) -> Tuple[int, int]:
    """BFS search to find the nearest free cell from (mx, my)."""
    fCache = FrontierCache()
    bfs = [fCache.getPoint(mx, my)]

    while len(bfs) > 0:
        loc = bfs.pop(0)
        if costmap.getCost(loc.mapX, loc.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value:
            return (loc.mapX, loc.mapY)
        for n in _getNeighbors(loc, costmap, fCache):
            if n.classification & PointClassification.MapClosed.value == 0:
                n.classification = n.classification | PointClassification.MapClosed.value
                bfs.append(n)

    return (mx, my)


def _getFrontier(
    pose_x: float, pose_y: float,
    costmap: OccupancyGrid2d,
    logger,
) -> List[Tuple[float, float, int]]:
    """
    Wavefront Frontier Detection (WFD) algorithm.
    Returns a list of (centroid_x, centroid_y, cluster_size) tuples in world coordinates.
    """
    fCache = FrontierCache()
    fCache.clear()

    try:
        mx, my = costmap.worldToMap(pose_x, pose_y)
    except Exception:
        logger.warn("Robot pose outside map bounds — skipping frontier search")
        return []

    freePoint = _findFree(mx, my, costmap)
    start = fCache.getPoint(freePoint[0], freePoint[1])
    start.classification = PointClassification.MapOpen.value
    mapPointQueue = [start]

    frontiers: List[Tuple[float, float, int]] = []

    while len(mapPointQueue) > 0:
        p = mapPointQueue.pop(0)

        if p.classification & PointClassification.MapClosed.value != 0:
            continue

        if _isFrontierPoint(p, costmap, fCache):
            p.classification = p.classification | PointClassification.FrontierOpen.value
            frontierQueue = [p]
            newFrontier: List[FrontierPoint] = []

            while len(frontierQueue) > 0:
                q = frontierQueue.pop(0)

                if q.classification & (
                    PointClassification.MapClosed.value
                    | PointClassification.FrontierClosed.value
                ) != 0:
                    continue

                if _isFrontierPoint(q, costmap, fCache):
                    newFrontier.append(q)

                    for w in _getNeighbors(q, costmap, fCache):
                        if w.classification & (
                            PointClassification.FrontierOpen.value
                            | PointClassification.FrontierClosed.value
                            | PointClassification.MapClosed.value
                        ) == 0:
                            w.classification = (
                                w.classification
                                | PointClassification.FrontierOpen.value
                            )
                            frontierQueue.append(w)

                q.classification = (
                    q.classification | PointClassification.FrontierClosed.value
                )

            newFrontierCoords = []
            for fp in newFrontier:
                fp.classification = (
                    fp.classification | PointClassification.MapClosed.value
                )
                newFrontierCoords.append(costmap.mapToWorld(fp.mapX, fp.mapY))

            if len(newFrontier) > 0:
                cx, cy = _centroid(newFrontierCoords)
                frontiers.append((cx, cy, len(newFrontier)))

        for v in _getNeighbors(p, costmap, fCache):
            if v.classification & (
                PointClassification.MapOpen.value
                | PointClassification.MapClosed.value
            ) == 0:
                if any(
                    costmap.getCost(x.mapX, x.mapY)
                    == OccupancyGrid2d.CostValues.FreeSpace.value
                    for x in _getNeighbors(v, costmap, fCache)
                ):
                    v.classification = (
                        v.classification | PointClassification.MapOpen.value
                    )
                    mapPointQueue.append(v)

        p.classification = p.classification | PointClassification.MapClosed.value

    return frontiers


# =============================================================================
#  State machine
# =============================================================================

class State(Enum):
    INIT_FORWARD    = auto()   # Drive forward briefly to seed SLAM (no spin)
    SELECT_FRONTIER = auto()   # Pick best frontier and dispatch Nav2 goal
    NAVIGATING      = auto()   # Waiting for Nav2 to reach goal
    AVOIDING        = auto()   # Back-up + gentle curve (no spin)
    RECOVERING      = auto()   # Slow forward drive to expose new frontiers
    COMPLETE        = auto()   # All frontiers exhausted


# =============================================================================
#  Frontier data class
# =============================================================================

class Frontier:
    __slots__ = ("cx", "cy", "size", "score")

    def __init__(self, cx: float, cy: float, size: int = 1) -> None:
        self.cx = cx
        self.cy = cy
        self.size = size
        self.score = 0.0


# =============================================================================
#  Main exploration node
# =============================================================================

class FrontierExplorer(Node):

    # -------------------------------------------------------------------------
    #  Init
    # -------------------------------------------------------------------------

    def __init__(self) -> None:
        super().__init__("frontier_explorer")

        self._declare_params()
        self._load_params()

        # Runtime state
        self.state: State = State.INIT_FORWARD
        self.map_data: Optional[OccupancyGrid] = None
        self.costmap_data: Optional[OccupancyGrid] = None
        self.latest_scan: Optional[LaserScan] = None
        self.latest_scan_time: Optional[float] = None
        self._enabled: bool = True

        # Robot pose (map frame)
        self.rx = self.ry = self.ryaw = 0.0

        # Init-forward timing
        self._fwd_t0: Optional[float] = None

        # Navigation bookkeeping
        self._goal_handle = None
        self._nav_t0: Optional[float] = None
        self._nav_done: bool = False
        self._nav_ok: bool = False

        # Counters
        self.consec_fail: int = 0
        self.total_fail: int = 0
        self.recov_spins: int = 0  # kept as counter name for compat

        # Avoidance bookkeeping
        self._avoid_t0: Optional[float] = None
        self._avoid_phase: int = 0  # 0=back-up, 1=curve
        self._avoid_direction: float = 1.0  # +1 = curve left, -1 = curve right
        self._last_front_obstacle_angle: float = 0.0
        self._last_front_obstacle_y: float = 0.0

        # Recovery bookkeeping
        self._recov_t0: Optional[float] = None

        # Visited frontier history (bounded deque)
        self._visited: Deque[Tuple[float, float]] = deque(maxlen=24)

        # Ensure COMPLETE executes exactly once
        self._completed: bool = False

        # Debounce for 360° safety perimeter triggers
        self._safety_hits: int = 0
        self._last_scan_warn: float = 0.0

        # Service readiness flags (updated by 1 Hz probe timer)
        self._svc_local_ok: bool = False
        self._svc_global_ok: bool = False

        # TF
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        # QoS: transient-local for map topics
        tl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # Subscriptions
        self.create_subscription(
            OccupancyGrid, "/map", self._map_cb, tl_qos)
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._costmap_cb, tl_qos)
        self.create_subscription(
            LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(
            Bool, "/explore/enable", self._enable_cb, 10)

        # Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, self.p_cmd_vel_topic, 10)
        self._viz_pub = self.create_publisher(MarkerArray, "/frontiers", 10)

        # Nav2 action client
        self._nav_ac = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Costmap clear service clients
        self._clear_local = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap")
        self._clear_global = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap")

        # Timers
        self._ctrl_timer = self.create_timer(0.2, self._ctrl_loop)
        self._prog_timer = self.create_timer(self.p_log_interval, self._log_progress)
        self._svc_timer = self.create_timer(1.0, self._probe_services)

        self.get_logger().info(
            "\n"
            "╔══════════════════════════════════════════════════╗\n"
            "║  Leo Rover Frontier Explorer  (ROS2 Jazzy)       ║\n"
            "║  v3.2 — WFD + body-aware obstacle avoidance      ║\n"
            "║  No-spin mode | Real lidar extrinsics supported  ║\n"
            "║  Publish False to /explore/enable to pause.       ║\n"
            "╚══════════════════════════════════════════════════╝"
        )
        self.get_logger().info(
            f"Velocity commands will be published on {self.p_cmd_vel_topic}"
        )
        self.get_logger().info(
            "Safety geometry: "
            f"laser=({self.p_laser_x_offset:.3f}, {self.p_laser_y_offset:.3f}, "
            f"yaw={math.degrees(self.p_laser_yaw_offset):.1f} deg), "
            f"body front/rear/half_width="
            f"{self.p_robot_front:.3f}/{self.p_robot_rear:.3f}/{self.p_robot_half_width:.3f} m, "
            f"clearance={self.p_body_clearance:.2f} m"
        )

    # -------------------------------------------------------------------------
    #  Parameters
    # -------------------------------------------------------------------------

    def _declare_params(self) -> None:
        self.declare_parameter("cmd_vel_topic",       "/cmd_vel")
        self.declare_parameter("robot_frame",          "base_link")
        self.declare_parameter("map_frame",            "map")
        self.declare_parameter("min_frontier_size",    5)
        self.declare_parameter("obstacle_dist",        0.20)    # clearance ahead of body, metres
        self.declare_parameter("scan_half_angle",      70.0)    # degrees around robot-forward
        self.declare_parameter("safety_radius",        0.10)    # legacy alias/logging for body clearance
        self.declare_parameter("body_clearance",       0.10)    # hard safety envelope outside body
        self.declare_parameter("self_filter_padding",  0.02)    # ignore points inside body + this margin
        self.declare_parameter("safety_self_clear_radius", 0.0)  # legacy parameter, no longer radial
        self.declare_parameter("safety_rear_exclusion_deg", 0.0) # optional rear blind wedge
        self.declare_parameter("safety_trigger_count",  2)       # debounce cycles before emergency avoid
        self.declare_parameter("scan_timeout",         0.7)      # stop if scan stream goes stale
        self.declare_parameter("front_min_points",      3)       # close points needed in front corridor
        self.declare_parameter("safety_min_points",     2)       # close points needed for perimeter stop
        self.declare_parameter("laser_x_offset",       0.0)      # scan origin in base_link
        self.declare_parameter("laser_y_offset",       0.0)
        self.declare_parameter("laser_yaw_offset",     0.0)      # radians; + means laser zero points left
        self.declare_parameter("robot_front",          0.2225)   # Leo 445 mm long
        self.declare_parameter("robot_rear",          -0.2225)
        self.declare_parameter("robot_half_width",     0.212)    # Leo 424 mm wide
        self.declare_parameter("nav_timeout",          35.0)    # s
        self.declare_parameter("init_forward_speed",   0.15)    # m/s
        self.declare_parameter("init_forward_duration", 3.0)    # s
        self.declare_parameter("backup_speed",        -0.14)    # m/s
        self.declare_parameter("backup_duration",      1.0)     # s
        self.declare_parameter("avoid_curve_speed",    0.08)    # m/s linear during curve
        self.declare_parameter("avoid_curve_angular",  0.35)    # rad/s during curve
        self.declare_parameter("avoid_curve_duration",  1.4)    # s
        self.declare_parameter("recov_forward_speed",  0.10)    # m/s
        self.declare_parameter("recov_forward_duration", 2.5)   # s
        self.declare_parameter("max_consec_fail",      4)
        self.declare_parameter("costmap_clear_every",  3)
        self.declare_parameter("complete_no_frontier", 8)
        self.declare_parameter("log_interval",        12.0)     # s
        self.declare_parameter("save_map_on_complete", True)
        self.declare_parameter("map_save_path",       "/tmp/leo_explored_map")

    def _load_params(self) -> None:
        g = self.get_parameter
        self.p_cmd_vel_topic  = g("cmd_vel_topic").value
        self.p_robot_frame     = g("robot_frame").value
        self.p_map_frame       = g("map_frame").value
        self.p_min_frontier    = g("min_frontier_size").value
        self.p_obs_dist        = g("obstacle_dist").value
        self.p_scan_half_angle = math.radians(g("scan_half_angle").value)
        self.p_safety_radius   = g("safety_radius").value
        self.p_body_clearance  = max(
            float(g("body_clearance").value),
            float(self.p_safety_radius),
        )
        self.p_self_filter_padding = float(g("self_filter_padding").value)
        self.p_safety_self_clear_radius = g("safety_self_clear_radius").value
        self.p_safety_rear_exclusion = math.radians(g("safety_rear_exclusion_deg").value)
        self.p_safety_trigger_count = int(g("safety_trigger_count").value)
        self.p_scan_timeout = float(g("scan_timeout").value)
        self.p_front_min_points = int(g("front_min_points").value)
        self.p_safety_min_points = int(g("safety_min_points").value)
        self.p_laser_x_offset = float(g("laser_x_offset").value)
        self.p_laser_y_offset = float(g("laser_y_offset").value)
        self.p_laser_yaw_offset = float(g("laser_yaw_offset").value)
        self.p_robot_front = float(g("robot_front").value)
        self.p_robot_rear = float(g("robot_rear").value)
        self.p_robot_half_width = float(g("robot_half_width").value)
        self.p_nav_timeout     = g("nav_timeout").value
        self.p_fwd_speed       = g("init_forward_speed").value
        self.p_fwd_duration    = g("init_forward_duration").value
        self.p_backup_speed    = g("backup_speed").value
        self.p_backup_dur      = g("backup_duration").value
        self.p_curve_speed     = g("avoid_curve_speed").value
        self.p_curve_angular   = g("avoid_curve_angular").value
        self.p_curve_dur       = g("avoid_curve_duration").value
        self.p_recov_speed     = g("recov_forward_speed").value
        self.p_recov_dur       = g("recov_forward_duration").value
        self.p_max_consec      = g("max_consec_fail").value
        self.p_clear_every     = g("costmap_clear_every").value
        self.p_no_front_done   = g("complete_no_frontier").value
        self.p_log_interval    = g("log_interval").value
        self.p_save_map        = g("save_map_on_complete").value
        self.p_map_path        = g("map_save_path").value

    # -------------------------------------------------------------------------
    #  Callbacks
    # -------------------------------------------------------------------------

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.map_data = msg

    def _costmap_cb(self, msg: OccupancyGrid) -> None:
        self.costmap_data = msg

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.latest_scan_time = self._now_sec()

    def _enable_cb(self, msg: Bool) -> None:
        """Publish std_msgs/Bool False to /explore/enable to pause."""
        was = self._enabled
        self._enabled = msg.data
        if not was and msg.data:
            self.get_logger().info("RESUMED via /explore/enable")
        elif was and not msg.data:
            self.get_logger().info("PAUSED via /explore/enable")
            self._stop()
            if self._goal_handle is not None:
                self._cancel_nav()

    # -------------------------------------------------------------------------
    #  Timing helper  (ROS clock — works with use_sim_time:=true)
    # -------------------------------------------------------------------------

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -------------------------------------------------------------------------
    #  Service availability probe (1 Hz, non-blocking)
    # -------------------------------------------------------------------------

    def _probe_services(self) -> None:
        self._svc_local_ok = self._clear_local.service_is_ready()
        self._svc_global_ok = self._clear_global.service_is_ready()

    # -------------------------------------------------------------------------
    #  Utility helpers
    # -------------------------------------------------------------------------

    def _robot_in_map(self) -> Optional[Tuple[float, float, float]]:
        try:
            t = self.tf_buf.lookup_transform(
                self.p_map_frame, self.p_robot_frame, rclpy.time.Time()
            )
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            return tx, ty, yaw
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _scan_is_fresh(self, now: Optional[float] = None) -> bool:
        if self.latest_scan is None or self.latest_scan_time is None:
            return False
        if now is None:
            now = self._now_sec()
        return (now - self.latest_scan_time) <= self.p_scan_timeout

    def _scan_points_robot_frame(self):
        """Return valid LaserScan points transformed into base_link coordinates."""
        scan = self.latest_scan
        if scan is None:
            return None

        ranges = np.array(scan.ranges, dtype=np.float32)
        laser_angles = (
            np.arange(len(ranges), dtype=np.float32) * scan.angle_increment
            + scan.angle_min
        )

        min_range = max(float(scan.range_min), 0.02)
        valid = np.isfinite(ranges) & (ranges >= min_range)
        if scan.range_max > 0.0:
            valid &= (ranges <= float(scan.range_max))

        if not np.any(valid):
            return None

        valid_ranges = ranges[valid]
        base_angles = laser_angles[valid] + self.p_laser_yaw_offset
        base_angles = np.arctan2(np.sin(base_angles), np.cos(base_angles))
        xs = self.p_laser_x_offset + valid_ranges * np.cos(base_angles)
        ys = self.p_laser_y_offset + valid_ranges * np.sin(base_angles)
        return xs, ys, valid_ranges, base_angles

    def _inside_body_rect(self, xs, ys, clearance: float):
        return (
            (xs <= self.p_robot_front + clearance)
            & (xs >= self.p_robot_rear - clearance)
            & (np.abs(ys) <= self.p_robot_half_width + clearance)
        )

    def _clearance_to_body(self, xs, ys):
        dx_front = xs - self.p_robot_front
        dx_rear = self.p_robot_rear - xs
        dx = np.maximum(np.maximum(dx_front, dx_rear), 0.0)
        dy = np.maximum(np.abs(ys) - self.p_robot_half_width, 0.0)
        return np.hypot(dx, dy)

    def _obstacle_in_sector(self) -> Tuple[bool, float]:
        """
        Body-aware front obstacle check.

        obstacle_dist is clearance ahead of the physical front bumper, not raw
        lidar range. Points are first transformed from the laser frame into
        base_link using the configured lidar offset/yaw.
        """
        points = self._scan_points_robot_frame()
        self._last_front_obstacle_angle = 0.0
        self._last_front_obstacle_y = 0.0
        if points is None:
            return False, float("inf")

        xs, ys, _ranges, base_angles = points
        self_mask = self._inside_body_rect(xs, ys, self.p_self_filter_padding)
        corridor_half_width = self.p_robot_half_width + self.p_body_clearance
        front_mask = (
            (xs > self.p_robot_front)
            & (np.abs(ys) <= corridor_half_width)
            & (np.abs(base_angles) <= self.p_scan_half_angle)
            & (~self_mask)
        )

        if not np.any(front_mask):
            return False, float("inf")

        front_clearances = xs[front_mask] - self.p_robot_front
        front_angles = base_angles[front_mask]
        front_ys = ys[front_mask]
        min_idx = int(np.argmin(front_clearances))
        min_clearance = float(front_clearances[min_idx])
        self._last_front_obstacle_angle = float(front_angles[min_idx])
        self._last_front_obstacle_y = float(front_ys[min_idx])
        close_count = int(np.sum(front_clearances < self.p_obs_dist))
        return close_count >= self.p_front_min_points, min_clearance

    def _check_safety_perimeter(self) -> Tuple[bool, float, float]:
        """
        Rectangular safety perimeter check around the real robot body.

        Returns (is_danger, min_clearance_from_body, danger_angle_rad).
        """
        points = self._scan_points_robot_frame()
        if points is None:
            return False, float("inf"), 0.0

        xs, ys, _ranges, base_angles = points
        safety_mask = self._inside_body_rect(xs, ys, self.p_body_clearance)
        self_mask = self._inside_body_rect(xs, ys, self.p_self_filter_padding)
        valid = safety_mask & (~self_mask)

        if self.p_safety_rear_exclusion > 0.0:
            rear_mask = np.abs(np.abs(base_angles) - math.pi) < self.p_safety_rear_exclusion
            valid &= (~rear_mask)

        if not np.any(valid):
            return False, float("inf"), 0.0

        clearances = self._clearance_to_body(xs[valid], ys[valid])
        danger_angles = base_angles[valid]
        min_idx = int(np.argmin(clearances))
        min_clearance = float(clearances[min_idx])
        min_angle = float(danger_angles[min_idx])
        close_count = int(np.sum(clearances <= self.p_body_clearance))
        return close_count >= self.p_safety_min_points, min_clearance, min_angle


    def _stop(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    def _drive(self, vx: float, wz: float = 0.0) -> None:
        t = Twist()
        t.linear.x = vx
        t.angular.z = wz
        self._cmd_vel_pub.publish(t)

    # -------------------------------------------------------------------------
    #  Map statistics
    # -------------------------------------------------------------------------

    def _map_stats(self) -> Tuple[int, int, int, float]:
        if self.map_data is None:
            return 0, 0, 0, 0.0
        d = np.array(self.map_data.data, dtype=np.int8)
        free = int(np.sum(d == 0))
        occ = int(np.sum((d > 0) & (d <= 100)))
        unk = int(np.sum(d == -1))
        pct = (free + occ) / d.size * 100.0
        return free, occ, unk, pct

    def _explored_pct(self) -> float:
        return self._map_stats()[3]

    # -------------------------------------------------------------------------
    #  Frontier detection — WFD algorithm
    # -------------------------------------------------------------------------

    def _detect_frontiers(self) -> List[Frontier]:
        """
        Run the Wavefront Frontier Detection (WFD) BFS algorithm on the
        current occupancy grid.  Returns scored Frontier objects.

        The WFD algorithm:
        1. Converts robot pose to map coordinates
        2. Finds nearest free cell via BFS
        3. Performs wavefront expansion from the free cell
        4. At each frontier point, traces the entire connected frontier cluster
        5. Returns centroids of clusters larger than MIN_FRONTIER_SIZE
        """
        if self.map_data is None:
            return []

        pose = self._robot_in_map()
        if pose is None:
            return []
        rx, ry, _ = pose

        # Wrap the raw OccupancyGrid in our WFD adapter
        og2d = OccupancyGrid2d(self.map_data)

        # Run WFD BFS
        frontier_clusters = _getFrontier(rx, ry, og2d, self.get_logger())

        # Convert WFD centroids to Frontier objects, applying cluster-size and inflation filters
        frontiers: List[Frontier] = []
        for (wx, wy, cluster_size) in frontier_clusters:
            if cluster_size < self.p_min_frontier:
                continue
            if self._frontier_in_inflation(wx, wy):
                continue
            frontiers.append(Frontier(wx, wy, size=cluster_size))

        return frontiers

    def _frontier_in_inflation(self, wx: float, wy: float) -> bool:
        """
        Return True if the world point falls inside an inflation or lethal
        zone according to the Nav2 global costmap.
        """
        cm = self.costmap_data
        if cm is None:
            return False

        res = cm.info.resolution
        ox = cm.info.origin.position.x
        oy = cm.info.origin.position.y
        w = cm.info.width
        h = cm.info.height

        col = int((wx - ox) / res)
        row = int((wy - oy) / res)

        if not (0 <= col < w and 0 <= row < h):
            return False

        cost = cm.data[row * w + col]
        return 0 <= cost < 255 and cost >= COSTMAP_LETHAL_THRESH

    # -------------------------------------------------------------------------
    #  Frontier scoring
    # -------------------------------------------------------------------------

    def _score_frontiers(
        self,
        frontiers: List[Frontier],
        rx: float, ry: float, ryaw: float,
    ) -> List[Frontier]:
        """
        score = 0.45 * info_gain
              + 0.35 * dist_score
              + 0.15 * dir_score
              - visit_penalty

        Frontiers closer than 0.5 m are filtered out.
        Direction score prefers frontiers AHEAD of robot heading (no rotation).
        """
        if len(self._visited) > 50:
            trimmed = list(self._visited)[-30:]
            self._visited.clear()
            self._visited.extend(trimmed)

        result: List[Frontier] = []
        for f in frontiers:
            dist = math.hypot(f.cx - rx, f.cy - ry)

            if dist < 0.5:
                continue

            # Information gain
            info_score = min(1.0, f.size / 60.0)

            # Distance: prefer 1.0-4.0 m range
            if dist <= 4.0:
                dist_score = 1.0 - abs(dist - 2.0) / 4.0
            else:
                dist_score = max(0.0, 1.0 - (dist - 4.0) / 6.0)

            # Direction: STRONGLY prefer frontiers ahead of robot heading
            # (since we never rotate, forward-facing targets are best)
            angle_to = math.atan2(f.cy - ry, f.cx - rx)
            angle_diff = abs(math.atan2(
                math.sin(angle_to - ryaw),
                math.cos(angle_to - ryaw),
            ))
            dir_score = max(0.0, 1.0 - angle_diff / math.pi)

            # Revisit penalty (anti ghost-wall)
            visit_pen = 0.0
            for vx, vy in self._visited:
                if math.hypot(f.cx - vx, f.cy - vy) < 0.6:
                    visit_pen = 0.40
                    break

            f.score = (
                0.45 * info_score
                + 0.35 * dist_score
                + 0.15 * dir_score
                - visit_pen
            )
            result.append(f)

        result.sort(key=lambda f: f.score, reverse=True)
        return result

    # -------------------------------------------------------------------------
    #  Navigation
    # -------------------------------------------------------------------------

    def _send_goal(self, fx: float, fy: float) -> bool:
        if not self._nav_ac.server_is_ready():
            self.get_logger().warn("Nav2 action server not ready - retry next cycle")
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.p_map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = fx
        goal.pose.pose.position.y = fy
        goal.pose.pose.orientation.w = 1.0

        self._nav_done = False
        self._nav_ok = False
        self._nav_t0 = self._now_sec()

        fut = self._nav_ac.send_goal_async(goal)
        fut.add_done_callback(self._goal_resp_cb)
        self.get_logger().info(f"Goal -> ({fx:.2f}, {fy:.2f})")
        return True

    def _goal_resp_cb(self, future) -> None:
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            self._nav_done = True
            self._nav_ok = False
            return
        self._goal_handle = gh
        gh.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        status = future.result().status
        self._nav_ok = (status == GoalStatus.STATUS_SUCCEEDED)
        self._nav_done = True
        self._goal_handle = None
        if self._nav_ok:
            self.get_logger().info("Navigation succeeded")
        else:
            self.get_logger().warn(f"Navigation failed (status={status})")

    def _cancel_nav(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._nav_done = True

    def _clear_costmaps(self) -> None:
        self.get_logger().info("Clearing costmaps...")
        req = ClearEntireCostmap.Request()
        if self._svc_local_ok:
            self._clear_local.call_async(req)
        else:
            self.get_logger().warn("Local costmap clear service not ready")
        if self._svc_global_ok:
            self._clear_global.call_async(req)
        else:
            self.get_logger().warn("Global costmap clear service not ready")

    # -------------------------------------------------------------------------
    #  Map save
    # -------------------------------------------------------------------------

    def _save_map(self) -> None:
        if not self.p_save_map:
            return
        path = self.p_map_path
        self.get_logger().info(f"Saving map to {path}.[yaml|pgm] ...")
        try:
            subprocess.Popen(
                [
                    "ros2", "run", "nav2_map_server", "map_saver_cli",
                    "-f", path,
                    "--ros-args", "-p", "save_map_timeout:=5.0",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except FileNotFoundError:
            self.get_logger().warn("map_saver_cli not found - skipping map save")

    # -------------------------------------------------------------------------
    #  Visualisation
    # -------------------------------------------------------------------------

    def _publish_frontiers(self, frontiers: List[Frontier]) -> None:
        ma = MarkerArray()
        del_m = Marker()
        del_m.action = Marker.DELETEALL
        ma.markers.append(del_m)

        stamp = self.get_clock().now().to_msg()
        for i, f in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = self.p_map_frame
            m.header.stamp = stamp
            m.ns = "frontiers"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = f.cx
            m.pose.position.y = f.cy
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            r_sz = 0.08 + 0.04 * min(1.0, f.size / 40.0)
            m.scale.x = r_sz * 2
            m.scale.y = r_sz * 2
            m.scale.z = 0.05
            s = max(0.0, min(1.0, f.score + 0.5))
            m.color.r = 1.0 - s
            m.color.g = s
            m.color.b = 0.2
            m.color.a = 0.85
            ma.markers.append(m)

        self._viz_pub.publish(ma)

    # -------------------------------------------------------------------------
    #  Logging
    # -------------------------------------------------------------------------

    def _log_progress(self) -> None:
        free, occ, unk, pct = self._map_stats()
        if free == occ == unk == 0:
            return
        self.get_logger().info(
            f"\n--- Exploration Progress ---\n"
            f"  State          : {self.state.name}\n"
            f"  Enabled        : {self._enabled}\n"
            f"  Explored       : {pct:.1f}%  "
            f"(free={free}, occ={occ}, unknown={unk})\n"
            f"  Consec failures: {self.consec_fail}  "
            f"Total failures: {self.total_fail}"
        )

    # -------------------------------------------------------------------------
    #  Control loop
    # -------------------------------------------------------------------------

    def _ctrl_loop(self) -> None:
        if not self._enabled:
            return

        now = self._now_sec()

        if self.state is not State.COMPLETE and not self._scan_is_fresh(now):
            self._stop()
            if now - self._last_scan_warn > 2.0:
                self.get_logger().warn(
                    "LaserScan stream is stale or unavailable - holding still"
                )
                self._last_scan_warn = now
            return

        # ── Global safety perimeter (runs in EVERY state except AVOIDING
        #    and COMPLETE) ── 360° check with debounce
        if self.state not in (State.AVOIDING, State.COMPLETE):
            danger, s_dist, s_angle = self._check_safety_perimeter()
            if danger:
                self._safety_hits += 1
            else:
                self._safety_hits = 0

            if self._safety_hits >= self.p_safety_trigger_count:
                self.get_logger().warn(
                    f"Safety perimeter breach! Body clearance {s_dist:.2f}m "
                    f"at angle={math.degrees(s_angle):.0f} deg - emergency avoidance"
                )
                self._stop()
                # Cancel any active Nav2 goal
                if self.state == State.NAVIGATING and self._goal_handle is not None:
                    self._cancel_nav()
                # Choose avoidance curve direction: steer AWAY from obstacle
                # Obstacle on left (angle > 0) → curve right (-1)
                # Obstacle on right (angle < 0) → curve left (+1)
                self._avoid_direction = -1.0 if s_angle > 0.0 else 1.0
                self._avoid_t0 = now
                self._avoid_phase = 0
                self._recov_t0 = None
                self._fwd_t0 = None
                self._safety_hits = 0
                self.state = State.AVOIDING
                return
        {
            State.INIT_FORWARD:    self._state_init_forward,
            State.SELECT_FRONTIER: self._state_select,
            State.NAVIGATING:      self._state_navigating,
            State.AVOIDING:        self._state_avoiding,
            State.RECOVERING:      self._state_recovering,
            State.COMPLETE:        self._state_complete,
        }[self.state](now)

    # -------------------------------------------------------------------------
    #  State: INIT_FORWARD  (replaces INIT_SPIN — no rotation)
    # -------------------------------------------------------------------------

    def _state_init_forward(self, now: float) -> None:
        """
        Drive straight forward briefly to seed the SLAM map.
        No rotation — lidar mount stays stable.
        """
        if self._fwd_t0 is None:
            self._fwd_t0 = now
            self.get_logger().info(
                f"Driving forward at {self.p_fwd_speed} m/s for "
                f"{self.p_fwd_duration}s to seed SLAM map (no spin)..."
            )

        # Safety check: if obstacle appears, skip init drive
        obs, dist = self._obstacle_in_sector()
        if obs:
            self.get_logger().warn(
                f"Obstacle clearance {dist:.2f}m during init drive - starting exploration"
            )
            self._stop()
            self._fwd_t0 = None
            self._safety_hits = 0
            self.state = State.SELECT_FRONTIER
            return

        if now - self._fwd_t0 < self.p_fwd_duration:
            self._drive(self.p_fwd_speed)
        else:
            self._stop()
            self._fwd_t0 = None
            self._safety_hits = 0
            self.get_logger().info("Init forward done — starting exploration")
            self.state = State.SELECT_FRONTIER

    # -------------------------------------------------------------------------
    #  State: SELECT_FRONTIER
    # -------------------------------------------------------------------------

    def _state_select(self, now: float) -> None:
        if self.map_data is None:
            self.get_logger().warn("Waiting for /map...")
            return

        # Periodic costmap clear on consecutive failures (non-blocking)
        if self.consec_fail > 0 and self.consec_fail % self.p_clear_every == 0:
            self._clear_costmaps()

        pose = self._robot_in_map()
        if pose is None:
            self.get_logger().warn("TF not ready (map->base_link)")
            return
        rx, ry, ryaw = pose
        self.rx, self.ry, self.ryaw = rx, ry, ryaw

        raw_frontiers = self._detect_frontiers()

        if not raw_frontiers:
            self.recov_spins += 1
            self.get_logger().warn(
                f"No frontiers found (streak={self.recov_spins})"
            )
            if self.recov_spins >= self.p_no_front_done:
                self.get_logger().info("No frontiers left — exploration complete!")
                self.state = State.COMPLETE
            else:
                self.state = State.RECOVERING
            return

        self.recov_spins = 0

        if self.consec_fail >= self.p_max_consec:
            self.get_logger().warn("Too many consecutive failures — recovery drive")
            self.consec_fail = 0
            self.state = State.RECOVERING
            return

        scored = self._score_frontiers(raw_frontiers, rx, ry, ryaw)
        self._publish_frontiers(scored[:20])

        # All frontiers too close (filtered out)
        if not scored:
            self.get_logger().warn(
                "All frontiers too close — driving forward to explore"
            )
            if self._fwd_t0 is None:
                self._fwd_t0 = self._now_sec()
            elapsed = self._now_sec() - self._fwd_t0
            if elapsed < 3.0:
                self._drive(0.15)
                return
            else:
                self._stop()
                self._fwd_t0 = None
                return

        best = scored[0]
        self.get_logger().info(
            f"Best frontier: ({best.cx:.2f}, {best.cy:.2f})  "
            f"size={best.size}  score={best.score:.3f}"
        )
        self._visited.append((best.cx, best.cy))

        if self._send_goal(best.cx, best.cy):
            self.state = State.NAVIGATING
        else:
            self.consec_fail += 1

    # -------------------------------------------------------------------------
    #  State: NAVIGATING
    # -------------------------------------------------------------------------

    def _state_navigating(self, now: float) -> None:
        # Emergency obstacle avoidance: body-aware front clearance check.
        obs, dist = self._obstacle_in_sector()
        if obs:
            self.get_logger().warn(
                f"Front body clearance {dist:.2f} m - emergency avoidance (no spin)"
            )
            self._cancel_nav()
            self._avoid_t0 = now
            self._avoid_phase = 0
            self._avoid_direction = -1.0 if self._last_front_obstacle_y > 0.0 else 1.0
            self.state = State.AVOIDING
            return

        # Navigation result arrived via callback
        if self._nav_done:
            if self._nav_ok:
                self.consec_fail = 0
            else:
                self.consec_fail += 1
                self.total_fail += 1
            self._nav_done = False
            self.state = State.SELECT_FRONTIER
            return

        # Timeout guard
        if (
            self._nav_t0 is not None
            and (now - self._nav_t0) > self.p_nav_timeout
        ):
            self.get_logger().warn("Navigation timeout — cancelling goal")
            self._cancel_nav()
            self.consec_fail += 1
            self.total_fail += 1
            self._nav_done = False
            self.state = State.SELECT_FRONTIER

    # -------------------------------------------------------------------------
    #  State: AVOIDING  (back-up + gentle curve — NO spin)
    # -------------------------------------------------------------------------

    def _state_avoiding(self, now: float) -> None:
        """
        Two-phase avoidance WITHOUT any rotation:
          Phase 0: Back up straight
          Phase 1: Curve gently (forward + slight turn) to clear obstacle
        """
        if self._avoid_phase == 0:
            # Phase 0: back up
            elapsed = now - (self._avoid_t0 or now)
            if elapsed < self.p_backup_dur:
                self._drive(self.p_backup_speed)
            else:
                # Transition to curve phase
                self._avoid_phase = 1
                self._avoid_t0 = now
                self.get_logger().info("Backup done — curving to clear obstacle")
        else:
            # Phase 1: gentle curve (forward + angular) — steer AWAY from obstacle
            elapsed = now - (self._avoid_t0 or now)
            if elapsed < self.p_curve_dur:
                self._drive(
                    self.p_curve_speed,
                    self.p_curve_angular * self._avoid_direction
                )
            else:
                self._stop()
                self._safety_hits = 0
                self.get_logger().info("Avoidance manoeuvre complete (no spin)")
                # Do not count every successful avoidance as a hard navigation failure.
                self.state = State.SELECT_FRONTIER


    # -------------------------------------------------------------------------
    #  State: RECOVERING  (slow forward drive — NO spin)
    # -------------------------------------------------------------------------

    def _state_recovering(self, now: float) -> None:
        """
        Drive forward slowly to expose new frontiers.
        No spinning — keeps lidar stable and avoids ghost walls.
        """
        if self._recov_t0 is None:
            self._recov_t0 = now
            self._safety_hits = 0
            self._clear_costmaps()
            self.get_logger().info(
                f"Recovery: driving forward at {self.p_recov_speed} m/s "
                f"for {self.p_recov_dur}s (no spin)"
            )

        # Safety: check for obstacles during recovery drive
        obs, dist = self._obstacle_in_sector()
        if obs:
            self.get_logger().warn(
                f"Front body clearance {dist:.2f}m during recovery - switching to avoidance"
            )
            self._stop()
            self._recov_t0 = None
            self._avoid_t0 = self._now_sec()
            self._avoid_phase = 0
            self._safety_hits = 0
            self.state = State.AVOIDING
            return

        if now - self._recov_t0 < self.p_recov_dur:
            self._drive(self.p_recov_speed)
        else:
            self._stop()
            self._recov_t0 = None
            self.get_logger().info("Recovery forward drive done")
            self.state = State.SELECT_FRONTIER

    # -------------------------------------------------------------------------
    #  State: COMPLETE  (exactly-once guard)
    # -------------------------------------------------------------------------

    def _state_complete(self, now: float) -> None:
        if self._completed:
            return
        self._completed = True

        self._stop()
        _, _, _, pct = self._map_stats()
        self.get_logger().info(
            f"\n=== EXPLORATION COMPLETE ===\n"
            f"  Explored area  : {pct:.1f}%\n"
            f"  Total failures : {self.total_fail}"
        )

        self._ctrl_timer.cancel()
        self._prog_timer.cancel()
        self._svc_timer.cancel()

        self._save_map()


# =============================================================================
#  Entry point
# =============================================================================

def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node._stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
