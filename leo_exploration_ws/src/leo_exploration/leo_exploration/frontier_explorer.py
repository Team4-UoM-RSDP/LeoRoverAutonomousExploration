#!/usr/bin/env python3
"""
Leo Rover Frontier-Based Autonomous Exploration Node
ROS2 Jazzy  |  v3.0  —  Wavefront Frontier Detection (WFD)

Core algorithm ported from nav2_wavefront_frontier_exploration
(arXiv:1806.03581).  BFS-based frontier detection on the raw
OccupancyGrid with Cantor-hash point cache and bit-flag
classification (MapOpen / MapClosed / FrontierOpen / FrontierClosed).

Infrastructure retained from v2.0:
  - State machine  (INIT_SPIN → SELECT_FRONTIER → NAVIGATING →
                     AVOIDING → RECOVERING → COMPLETE)
  - Async NavigateToPose (non-blocking, keeps obstacle avoidance live)
  - TF2 robot pose lookup
  - LaserScan emergency obstacle avoidance
  - Costmap inflation filter on frontiers
  - Costmap clearing on repeated failures
  - Map auto-save on COMPLETE
  - Pause / resume via Bool topic /explore/enable
  - RViz frontier marker visualisation
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

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

# ============================================================================
#  WFD Constants
# ============================================================================

OCC_THRESHOLD = 10          # cells with occupancy > this are near obstacles
MIN_FRONTIER_SIZE = 5       # minimum cells in a frontier cluster
COSTMAP_LETHAL_THRESH = 70  # nav2 costmap scale 0-100


# ============================================================================
#  WFD Support Classes  (from nav2_wavefront_frontier_exploration)
# ============================================================================

class OccupancyGrid2d:
    """Thin wrapper around nav_msgs/OccupancyGrid for grid access."""

    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 100
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map_msg):
        self.map = map_msg

    def getCost(self, mx, my):
        return self.map.data[self._getIndex(mx, my)]

    def getSize(self):
        return (self.map.info.width, self.map.info.height)

    def getSizeX(self):
        return self.map.info.width

    def getSizeY(self):
        return self.map.info.height

    def mapToWorld(self, mx, my):
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution
        return (wx, wy)

    def worldToMap(self, wx, wy):
        if (wx < self.map.info.origin.position.x or
                wy < self.map.info.origin.position.y):
            raise Exception("World coordinates out of bounds")
        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)
        if my > self.map.info.height or mx > self.map.info.width:
            raise Exception("Out of bounds")
        return (mx, my)

    def _getIndex(self, mx, my):
        return my * self.map.info.width + mx


class FrontierCache:
    """Cantor-hash based point cache for BFS visited tracking."""

    def __init__(self):
        self.cache = {}

    def getPoint(self, x, y):
        idx = self._cantorHash(x, y)
        if idx in self.cache:
            return self.cache[idx]
        self.cache[idx] = FrontierPoint(x, y)
        return self.cache[idx]

    def _cantorHash(self, x, y):
        return (((x + y) * (x + y + 1)) / 2) + y

    def clear(self):
        self.cache = {}


class FrontierPoint:
    """Single grid cell with bit-flag classification for BFS."""
    __slots__ = ("classification", "mapX", "mapY")

    def __init__(self, x, y):
        self.classification = 0
        self.mapX = x
        self.mapY = y


class PointClassification(Enum):
    MapOpen = 1
    MapClosed = 2
    FrontierOpen = 4
    FrontierClosed = 8


# ============================================================================
#  WFD Algorithm Functions  (from nav2_wavefront_frontier_exploration)
# ============================================================================

def centroid(arr):
    """Compute the centroid of a list of (x, y) world coordinates."""
    arr = np.array(arr)
    length = arr.shape[0]
    sum_x = np.sum(arr[:, 0])
    sum_y = np.sum(arr[:, 1])
    return sum_x / length, sum_y / length


def findFree(mx, my, costmap):
    """BFS from (mx, my) to find the nearest free-space cell."""
    fCache = FrontierCache()
    bfs = [fCache.getPoint(mx, my)]
    while len(bfs) > 0:
        loc = bfs.pop(0)
        if costmap.getCost(loc.mapX, loc.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value:
            return (loc.mapX, loc.mapY)
        for n in getNeighbors(loc, costmap, fCache):
            if n.classification & PointClassification.MapClosed.value == 0:
                n.classification = n.classification | PointClassification.MapClosed.value
                bfs.append(n)
    return (mx, my)


def getFrontier(pose, costmap, logger):
    """
    Wavefront Frontier Detection (WFD) algorithm.

    BFS on the occupancy grid starting from the robot's position.
    Finds all frontier regions (unknown cells adjacent to free cells)
    and returns the centroid of each frontier cluster in world coordinates.

    Args:
        pose: geometry_msgs/Pose with robot position
        costmap: OccupancyGrid2d wrapper
        logger: ROS2 logger for debug output

    Returns:
        List of (wx, wy) centroid tuples for each detected frontier
    """
    fCache = FrontierCache()
    fCache.clear()

    try:
        mx, my = costmap.worldToMap(pose.position.x, pose.position.y)
    except Exception:
        logger.warn("Robot position out of map bounds for WFD")
        return []

    freePoint = findFree(mx, my, costmap)
    start = fCache.getPoint(freePoint[0], freePoint[1])
    start.classification = PointClassification.MapOpen.value
    mapPointQueue = [start]

    frontiers = []

    while len(mapPointQueue) > 0:
        p = mapPointQueue.pop(0)

        if p.classification & PointClassification.MapClosed.value != 0:
            continue

        if isFrontierPoint(p, costmap, fCache):
            p.classification = p.classification | PointClassification.FrontierOpen.value
            frontierQueue = [p]
            newFrontier = []

            while len(frontierQueue) > 0:
                q = frontierQueue.pop(0)

                if q.classification & (PointClassification.MapClosed.value |
                                       PointClassification.FrontierClosed.value) != 0:
                    continue

                if isFrontierPoint(q, costmap, fCache):
                    newFrontier.append(q)

                    for w in getNeighbors(q, costmap, fCache):
                        if w.classification & (
                            PointClassification.FrontierOpen.value |
                            PointClassification.FrontierClosed.value |
                            PointClassification.MapClosed.value
                        ) == 0:
                            w.classification = (
                                w.classification | PointClassification.FrontierOpen.value
                            )
                            frontierQueue.append(w)

                q.classification = q.classification | PointClassification.FrontierClosed.value

            newFrontierCords = []
            for x in newFrontier:
                x.classification = x.classification | PointClassification.MapClosed.value
                newFrontierCords.append(costmap.mapToWorld(x.mapX, x.mapY))

            if len(newFrontier) > MIN_FRONTIER_SIZE:
                frontiers.append(centroid(newFrontierCords))

        for v in getNeighbors(p, costmap, fCache):
            if v.classification & (
                PointClassification.MapOpen.value |
                PointClassification.MapClosed.value
            ) == 0:
                if any(
                    costmap.getCost(x.mapX, x.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value
                    for x in getNeighbors(v, costmap, fCache)
                ):
                    v.classification = v.classification | PointClassification.MapOpen.value
                    mapPointQueue.append(v)

        p.classification = p.classification | PointClassification.MapClosed.value

    return frontiers


def getNeighbors(point, costmap, fCache):
    """Return 8-connected neighbors within map bounds."""
    neighbors = []
    for x in range(point.mapX - 1, point.mapX + 2):
        for y in range(point.mapY - 1, point.mapY + 2):
            if (x > 0 and x < costmap.getSizeX() and
                    y > 0 and y < costmap.getSizeY()):
                neighbors.append(fCache.getPoint(x, y))
    return neighbors


def isFrontierPoint(point, costmap, fCache):
    """
    A frontier point is an unknown cell that has at least one
    free-space neighbor, and NO neighbor above OCC_THRESHOLD.
    """
    if costmap.getCost(point.mapX, point.mapY) != OccupancyGrid2d.CostValues.NoInformation.value:
        return False

    hasFree = False
    for n in getNeighbors(point, costmap, fCache):
        cost = costmap.getCost(n.mapX, n.mapY)
        if cost > OCC_THRESHOLD:
            return False
        if cost == OccupancyGrid2d.CostValues.FreeSpace.value:
            hasFree = True

    return hasFree


# ============================================================================
#  State Machine
# ============================================================================

class State(Enum):
    INIT_SPIN       = auto()   # Initial 360 spin to seed SLAM map
    SELECT_FRONTIER = auto()   # Pick best frontier and dispatch Nav2 goal
    NAVIGATING      = auto()   # Waiting for Nav2 to reach goal
    AVOIDING        = auto()   # Composite avoidance: back-up then spin
    RECOVERING      = auto()   # Slow spin to expose new frontiers
    COMPLETE        = auto()   # All frontiers exhausted


# ============================================================================
#  Frontier data class (for scoring & visualisation)
# ============================================================================

class Frontier:
    __slots__ = ("cx", "cy", "score")

    def __init__(self, cx: float, cy: float) -> None:
        self.cx    = cx
        self.cy    = cy
        self.score = 0.0


# ============================================================================
#  Main Exploration Node
# ============================================================================

class FrontierExplorer(Node):

    # -------------------------------------------------------------------------
    #  Init
    # -------------------------------------------------------------------------

    def __init__(self) -> None:
        super().__init__("frontier_explorer")

        self._declare_params()
        self._load_params()

        # Runtime state
        self.state: State                           = State.INIT_SPIN
        self.map_data: Optional[OccupancyGrid]      = None
        self.costmap_data: Optional[OccupancyGrid]  = None
        self.latest_scan: Optional[LaserScan]       = None
        self._enabled: bool                         = True

        # Robot pose (map frame)
        self.rx = self.ry = self.ryaw = 0.0

        # Init-spin timing
        self._spin_t0: Optional[float] = None

        # Navigation bookkeeping
        self._goal_handle              = None
        self._nav_t0: Optional[float]  = None
        self._nav_done: bool           = False
        self._nav_ok:   bool           = False

        # Counters
        self.consec_fail:  int = 0
        self.total_fail:   int = 0
        self.recov_spins:  int = 0

        # Avoidance bookkeeping
        self._avoid_t0: Optional[float] = None
        self._avoid_phase: int          = 0   # 0=back-up, 1=spin

        # Recovery bookkeeping
        self._recov_t0: Optional[float] = None

        # Visited frontier history (bounded deque)
        self._visited: Deque[Tuple[float, float]] = deque(maxlen=24)

        # Completion guard (exactly-once)
        self._completed: bool = False

        # Service readiness flags (updated by 1 Hz probe timer)
        self._svc_local_ok:  bool = False
        self._svc_global_ok: bool = False

        # WFD costmap wrapper (rebuilt on each /map callback)
        self._wfd_costmap: Optional[OccupancyGrid2d] = None

        # TF
        self.tf_buf      = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        # QoS: transient-local for map topics
        tl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # Subscriptions
        self.create_subscription(OccupancyGrid, "/map",
                                 self._map_cb,     tl_qos)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 self._costmap_cb, tl_qos)
        self.create_subscription(LaserScan, "/scan",
                                 self._scan_cb, 10)
        self.create_subscription(Bool, "/explore/enable",
                                 self._enable_cb, 10)

        # Publishers
        self._cmd_vel_pub = self.create_publisher(Twist,       "/cmd_vel",   10)
        self._viz_pub     = self.create_publisher(MarkerArray, "/frontiers", 10)

        # Nav2 action client
        self._nav_ac = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Costmap clear service clients
        self._clear_local  = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap")
        self._clear_global = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap")

        # Timers
        self._ctrl_timer = self.create_timer(0.2,                  self._ctrl_loop)
        self._prog_timer = self.create_timer(self.p_log_interval,  self._log_progress)
        self._svc_timer  = self.create_timer(1.0,                  self._probe_services)

        self.get_logger().info(
            "\n"
            "╔══════════════════════════════════════════════════╗\n"
            "║  Leo Rover Frontier Explorer  (ROS2 Jazzy)       ║\n"
            "║  v3.0  —  Wavefront Frontier Detection (WFD)     ║\n"
            "║  Algorithm: arXiv:1806.03581                      ║\n"
            "║  Publish False to /explore/enable to pause.       ║\n"
            "╚══════════════════════════════════════════════════╝"
        )

    # -------------------------------------------------------------------------
    #  Parameters
    # -------------------------------------------------------------------------

    def _declare_params(self) -> None:
        self.declare_parameter("robot_frame",          "base_link")
        self.declare_parameter("map_frame",            "map")
        self.declare_parameter("min_frontier_size",    5)
        self.declare_parameter("occ_threshold",        10)
        self.declare_parameter("frontier_selection",   "nearest")     # nearest / farthest
        self.declare_parameter("obstacle_dist",        0.45)
        self.declare_parameter("obstacle_half_angle",  50.0)          # degrees
        self.declare_parameter("nav_timeout",          35.0)          # s
        self.declare_parameter("spin_speed",           0.55)          # rad/s
        self.declare_parameter("spin_duration",        12.5)          # s
        self.declare_parameter("backup_speed",        -0.18)          # m/s
        self.declare_parameter("backup_duration",      1.8)           # s
        self.declare_parameter("avoid_spin_duration",  2.5)           # s
        self.declare_parameter("recov_spin_duration",  7.0)           # s
        self.declare_parameter("max_consec_fail",      4)
        self.declare_parameter("costmap_clear_every",  3)
        self.declare_parameter("complete_no_frontier", 8)
        self.declare_parameter("log_interval",        12.0)           # s
        self.declare_parameter("save_map_on_complete", True)
        self.declare_parameter("map_save_path",       "/tmp/leo_explored_map")

    def _load_params(self) -> None:
        g = self.get_parameter
        self.p_robot_frame    = g("robot_frame").value
        self.p_map_frame      = g("map_frame").value
        self.p_min_frontier   = g("min_frontier_size").value
        self.p_occ_threshold  = g("occ_threshold").value
        self.p_frontier_sel   = g("frontier_selection").value
        self.p_obs_dist       = g("obstacle_dist").value
        self.p_obs_half_angle = math.radians(g("obstacle_half_angle").value)
        self.p_nav_timeout    = g("nav_timeout").value
        self.p_spin_speed     = g("spin_speed").value
        self.p_spin_duration  = g("spin_duration").value
        self.p_backup_speed   = g("backup_speed").value
        self.p_backup_dur     = g("backup_duration").value
        self.p_avoid_spin_dur = g("avoid_spin_duration").value
        self.p_recov_spin_dur = g("recov_spin_duration").value
        self.p_max_consec     = g("max_consec_fail").value
        self.p_clear_every    = g("costmap_clear_every").value
        self.p_no_front_done  = g("complete_no_frontier").value
        self.p_log_interval   = g("log_interval").value
        self.p_save_map       = g("save_map_on_complete").value
        self.p_map_path       = g("map_save_path").value

    # -------------------------------------------------------------------------
    #  Callbacks
    # -------------------------------------------------------------------------

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.map_data = msg
        self._wfd_costmap = OccupancyGrid2d(msg)

    def _costmap_cb(self, msg: OccupancyGrid) -> None:
        self.costmap_data = msg

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg

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
    #  Service availability probe  (1 Hz, non-blocking)
    # -------------------------------------------------------------------------

    def _probe_services(self) -> None:
        self._svc_local_ok  = self._clear_local.service_is_ready()
        self._svc_global_ok = self._clear_global.service_is_ready()

    # -------------------------------------------------------------------------
    #  Utility helpers
    # -------------------------------------------------------------------------

    def _robot_in_map(self) -> Optional[Tuple[float, float, float]]:
        try:
            t   = self.tf_buf.lookup_transform(
                self.p_map_frame, self.p_robot_frame, rclpy.time.Time()
            )
            tx  = t.transform.translation.x
            ty  = t.transform.translation.y
            q   = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            return tx, ty, yaw
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _obstacle_in_sector(self) -> Tuple[bool, float]:
        """Numpy-vectorised forward obstacle check."""
        scan = self.latest_scan
        if scan is None:
            return False, float("inf")
        ranges = np.array(scan.ranges, dtype=np.float32)
        angles = (
            np.arange(len(ranges), dtype=np.float32) * scan.angle_increment
            + scan.angle_min
        )
        valid = (
            np.isfinite(ranges)
            & (ranges > 0.01)
            & (np.abs(angles) <= self.p_obs_half_angle)
        )
        if not np.any(valid):
            return False, float("inf")
        min_d = float(np.min(ranges[valid]))
        return min_d < self.p_obs_dist, min_d

    def _stop(self)  -> None: self._cmd_vel_pub.publish(Twist())

    def _spin_cmd(self, speed: Optional[float] = None) -> None:
        t = Twist()
        t.angular.z = speed if speed is not None else self.p_spin_speed
        self._cmd_vel_pub.publish(t)

    def _drive(self, vx: float, wz: float = 0.0) -> None:
        t = Twist()
        t.linear.x  = vx
        t.angular.z = wz
        self._cmd_vel_pub.publish(t)

    def _drive_forward(self, speed: float = 0.15, duration: float = 3.0) -> None:
        """Drive forward for `duration` seconds, then return to SELECT_FRONTIER."""
        if not hasattr(self, '_fwd_t0') or self._fwd_t0 is None:
            self._fwd_t0 = self._now_sec()
            self.get_logger().info(f"Driving forward at {speed} m/s for {duration}s")
        elapsed = self._now_sec() - self._fwd_t0
        if elapsed < duration:
            self._drive(speed)
        else:
            self._stop()
            self._fwd_t0 = None
            self.state = State.SELECT_FRONTIER

    # -------------------------------------------------------------------------
    #  Map statistics
    # -------------------------------------------------------------------------

    def _map_stats(self) -> Tuple[int, int, int, float]:
        if self.map_data is None:
            return 0, 0, 0, 0.0
        d    = np.array(self.map_data.data, dtype=np.int8)
        free = int(np.sum(d == 0))
        occ  = int(np.sum((d > 0) & (d <= 100)))
        unk  = int(np.sum(d == -1))
        pct  = (free + occ) / d.size * 100.0
        return free, occ, unk, pct

    def _explored_pct(self) -> float:
        return self._map_stats()[3]

    # -------------------------------------------------------------------------
    #  WFD Frontier Detection  (wrapper around algorithm functions)
    # -------------------------------------------------------------------------

    def _detect_frontiers_wfd(self, rx: float, ry: float) -> List[Frontier]:
        """
        Run the Wavefront Frontier Detection algorithm on the current map.
        Returns a list of Frontier objects with world-coordinate centroids.
        """
        if self._wfd_costmap is None:
            return []

        # Update global constants from parameters
        global MIN_FRONTIER_SIZE, OCC_THRESHOLD
        MIN_FRONTIER_SIZE = self.p_min_frontier
        OCC_THRESHOLD = self.p_occ_threshold

        # Build a simple pose object for the WFD function
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x = rx
        pose.position.y = ry

        # Run WFD algorithm
        try:
            raw_frontiers = getFrontier(pose, self._wfd_costmap, self.get_logger())
        except Exception as e:
            self.get_logger().warn(f"WFD exception: {e}")
            return []

        # Convert to Frontier objects and apply costmap inflation filter
        frontiers: List[Frontier] = []
        for (wx, wy) in raw_frontiers:
            if self._frontier_in_inflation(wx, wy):
                continue
            frontiers.append(Frontier(wx, wy))

        return frontiers

    def _frontier_in_inflation(self, wx: float, wy: float) -> bool:
        """
        Return True if the world point (wx, wy) falls inside an inflation
        or lethal zone according to the Nav2 global costmap.
        """
        cm = self.costmap_data
        if cm is None:
            return False

        res = cm.info.resolution
        ox  = cm.info.origin.position.x
        oy  = cm.info.origin.position.y
        w   = cm.info.width
        h   = cm.info.height

        col = int((wx - ox) / res)
        row = int((wy - oy) / res)

        if not (0 <= col < w and 0 <= row < h):
            return False

        cost = cm.data[row * w + col]
        return 0 <= cost < 255 and cost >= COSTMAP_LETHAL_THRESH

    # -------------------------------------------------------------------------
    #  Frontier Scoring
    # -------------------------------------------------------------------------

    def _score_frontiers(
        self,
        frontiers: List[Frontier],
        rx: float, ry: float, ryaw: float,
    ) -> List[Frontier]:
        """
        Score frontiers based on distance and visited history.
        Supports 'nearest' and 'farthest' selection strategies.
        Frontiers closer than 0.5m are filtered out.
        """
        # Keep visited list bounded
        if len(self._visited) > 50:
            trimmed = list(self._visited)[-30:]
            self._visited.clear()
            self._visited.extend(trimmed)

        result: List[Frontier] = []
        for f in frontiers:
            dist = math.hypot(f.cx - rx, f.cy - ry)

            # Skip frontiers too close — they cause instant "reached goal"
            if dist < 0.5:
                continue

            # Distance score depends on selection strategy
            if self.p_frontier_sel == "farthest":
                # Prefer far frontiers (like original WFD)
                dist_score = min(1.0, dist / 10.0)
            else:
                # Prefer near frontiers (more efficient exploration)
                if dist <= 4.0:
                    dist_score = 1.0 - abs(dist - 2.0) / 4.0
                else:
                    dist_score = max(0.0, 1.0 - (dist - 4.0) / 6.0)

            # Direction: prefer frontiers roughly ahead of robot heading
            angle_to   = math.atan2(f.cy - ry, f.cx - rx)
            angle_diff = abs(math.atan2(
                math.sin(angle_to - ryaw),
                math.cos(angle_to - ryaw),
            ))
            dir_score = max(0.0, 1.0 - angle_diff / math.pi)

            # Revisit penalty
            visit_pen = 0.0
            for vx, vy in self._visited:
                if math.hypot(f.cx - vx, f.cy - vy) < 0.6:
                    visit_pen = 0.40
                    break

            f.score = (
                0.50 * dist_score
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
        goal.pose.header.frame_id    = self.p_map_frame
        goal.pose.header.stamp       = self.get_clock().now().to_msg()
        goal.pose.pose.position.x    = fx
        goal.pose.pose.position.y    = fy
        goal.pose.pose.orientation.w = 1.0

        self._nav_done = False
        self._nav_ok   = False
        self._nav_t0   = self._now_sec()

        fut = self._nav_ac.send_goal_async(goal)
        fut.add_done_callback(self._goal_resp_cb)
        self.get_logger().info(f"Goal -> ({fx:.2f}, {fy:.2f})")
        return True

    def _goal_resp_cb(self, future) -> None:
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            self._nav_done = True
            self._nav_ok   = False
            return
        self._goal_handle = gh
        gh.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        status        = future.result().status
        self._nav_ok  = (status == GoalStatus.STATUS_SUCCEEDED)
        self._nav_done     = True
        self._goal_handle  = None
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
        """Non-blocking costmap clearing."""
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
        ma    = MarkerArray()
        del_m = Marker()
        del_m.action = Marker.DELETEALL
        ma.markers.append(del_m)

        stamp = self.get_clock().now().to_msg()
        for i, f in enumerate(frontiers):
            m                    = Marker()
            m.header.frame_id    = self.p_map_frame
            m.header.stamp       = stamp
            m.ns                 = "frontiers"
            m.id                 = i
            m.type               = Marker.CYLINDER
            m.action             = Marker.ADD
            m.pose.position.x    = f.cx
            m.pose.position.y    = f.cy
            m.pose.position.z    = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = 0.16
            m.scale.y = 0.16
            m.scale.z = 0.05
            s          = max(0.0, min(1.0, f.score + 0.5))
            m.color.r  = 1.0 - s
            m.color.g  = s
            m.color.b  = 0.2
            m.color.a  = 0.85
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
            f"\n--- Exploration Progress (WFD v3.0) ---\n"
            f"  State          : {self.state.name}\n"
            f"  Enabled        : {self._enabled}\n"
            f"  Selection      : {self.p_frontier_sel}\n"
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
        {
            State.INIT_SPIN:       self._state_init_spin,
            State.SELECT_FRONTIER: self._state_select,
            State.NAVIGATING:      self._state_navigating,
            State.AVOIDING:        self._state_avoiding,
            State.RECOVERING:      self._state_recovering,
            State.COMPLETE:        self._state_complete,
        }[self.state](now)

    # -------------------------------------------------------------------------
    #  State: INIT_SPIN
    # -------------------------------------------------------------------------

    def _state_init_spin(self, now: float) -> None:
        if self._spin_t0 is None:
            self._spin_t0 = now
            self.get_logger().info("Initial 360 spin started...")

        if now - self._spin_t0 < self.p_spin_duration:
            self._spin_cmd()
        else:
            self._stop()
            self._spin_t0 = None
            self.get_logger().info("Initial spin done - starting WFD exploration")
            self.state = State.SELECT_FRONTIER

    # -------------------------------------------------------------------------
    #  State: SELECT_FRONTIER  (uses WFD algorithm)
    # -------------------------------------------------------------------------

    def _state_select(self, now: float) -> None:
        if self.map_data is None:
            self.get_logger().warn("Waiting for /map...")
            return

        # Periodic costmap clear on consecutive failures
        if self.consec_fail > 0 and self.consec_fail % self.p_clear_every == 0:
            self._clear_costmaps()

        pose = self._robot_in_map()
        if pose is None:
            self.get_logger().warn("TF not ready (map->base_link)")
            return
        rx, ry, ryaw = pose
        self.rx, self.ry, self.ryaw = rx, ry, ryaw

        # Run WFD algorithm
        raw_frontiers = self._detect_frontiers_wfd(rx, ry)

        if not raw_frontiers:
            self.recov_spins += 1
            self.get_logger().warn(
                f"No frontiers found by WFD (streak={self.recov_spins})"
            )
            if self.recov_spins >= self.p_no_front_done:
                self.get_logger().info("No frontiers left - exploration complete!")
                self.state = State.COMPLETE
            else:
                self.state = State.RECOVERING
            return

        self.recov_spins = 0

        if self.consec_fail >= self.p_max_consec:
            self.get_logger().warn("Too many consecutive failures - recovery spin")
            self.consec_fail = 0
            self.state = State.RECOVERING
            return

        scored = self._score_frontiers(raw_frontiers, rx, ry, ryaw)
        self._publish_frontiers(scored[:20])

        # All frontiers too close (filtered out by _score_frontiers)
        if not scored:
            self.get_logger().warn(
                "All frontiers too close — driving forward to explore"
            )
            self._drive_forward(speed=0.15, duration=3.0)
            return

        best = scored[0]
        self.get_logger().info(
            f"[WFD] Best frontier: ({best.cx:.2f}, {best.cy:.2f})  "
            f"score={best.score:.3f}  strategy={self.p_frontier_sel}"
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
        # Emergency obstacle avoidance (threshold = 65% of configured distance)
        obs, dist = self._obstacle_in_sector()
        if obs and dist < self.p_obs_dist * 0.65:
            self.get_logger().warn(
                f"Obstacle at {dist:.2f} m - emergency avoidance"
            )
            self._cancel_nav()
            self._avoid_t0    = now
            self._avoid_phase = 0
            self.state = State.AVOIDING
            return

        # Navigation result arrived via callback
        if self._nav_done:
            if self._nav_ok:
                self.consec_fail = 0
            else:
                self.consec_fail += 1
                self.total_fail  += 1
            self._nav_done = False
            self.state = State.SELECT_FRONTIER
            return

        # Timeout guard
        if (
            self._nav_t0 is not None
            and (now - self._nav_t0) > self.p_nav_timeout
        ):
            self.get_logger().warn("Navigation timeout - cancelling goal")
            self._cancel_nav()
            self.consec_fail += 1
            self.total_fail  += 1
            self._nav_done    = False
            self.state = State.SELECT_FRONTIER

    # -------------------------------------------------------------------------
    #  State: AVOIDING
    # -------------------------------------------------------------------------

    def _state_avoiding(self, now: float) -> None:
        if self._avoid_phase == 0:
            # Phase 0: back up
            elapsed = now - (self._avoid_t0 or now)
            if elapsed < self.p_backup_dur:
                self._drive(self.p_backup_speed)
            else:
                self._avoid_phase = 1
                self._avoid_t0    = now
                self._spin_cmd(self.p_spin_speed * 1.4)
        else:
            # Phase 1: spin to clear obstacle
            elapsed = now - (self._avoid_t0 or now)
            if elapsed < self.p_avoid_spin_dur:
                self._spin_cmd(self.p_spin_speed * 1.4)
            else:
                self._stop()
                self.get_logger().info("Avoidance manoeuvre complete")
                self.consec_fail += 1
                self.state = State.SELECT_FRONTIER

    # -------------------------------------------------------------------------
    #  State: RECOVERING
    # -------------------------------------------------------------------------

    def _state_recovering(self, now: float) -> None:
        if self._recov_t0 is None:
            self._recov_t0 = now
            self.get_logger().info("Recovery spin...")

        if now - self._recov_t0 < self.p_recov_spin_dur:
            self._spin_cmd()
        else:
            self._stop()
            self._recov_t0 = None
            self.get_logger().info("Recovery spin done")
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
            f"\n=== EXPLORATION COMPLETE (WFD v3.0) ===\n"
            f"  Explored area  : {pct:.1f}%\n"
            f"  Total failures : {self.total_fail}"
        )

        # Cancel all timers cleanly
        self._ctrl_timer.cancel()
        self._prog_timer.cancel()
        self._svc_timer.cancel()

        self._save_map()


# ============================================================================
#  Entry point
# ============================================================================

def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
