#!/usr/bin/env python3
"""
Leo Rover Frontier-Based Autonomous Exploration Node
ROS2 Jazzy  |  v3.0

Changelog v3.0 (ghost-wall fix & roomba-style exploration):
  - REMOVED: INIT_SPIN state — no initial 360° spin; exploration begins
    immediately.  This prevents the symmetric scan pattern that confuses
    SLAM loop closure when transitioning to a new area.
  - REMOVED: RECOVERING spin — replaced by WANDERING state that drives
    forward with a gentle random arc (like a robot vacuum).
  - REMOVED: Pure in-place spins everywhere — AVOIDING now uses arc-turns
    (backup + forward-arc) instead of backup + spin.
  - ADDED: 120° front-facing LiDAR filter — only the forward ±60° arc is
    published on /scan_filtered for SLAM; rear data is discarded to prevent
    conflicting scan-match when the robot reverses direction.
  - CHANGED: Obstacle detection uses the filtered 120° arc as well.
  - CHANGED: Nav2 goals are sent with orientation=robot's current heading
    so the controller never commands a large in-place rotation on arrival.
  - CHANGED: Frontier scoring direction weight increased (0.25 → 0.35) to
    favour frontiers ahead of the robot, reducing the need for sharp turns.
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

# Costmap threshold: cells with cost >= this are in inflation / lethal zone
COSTMAP_LETHAL_THRESH = 70   # range 0-100 (nav2 scale)


# =============================================================================
#  State machine
# =============================================================================

class State(Enum):
    SELECT_FRONTIER = auto()   # Pick best frontier and dispatch Nav2 goal
    NAVIGATING      = auto()   # Waiting for Nav2 to reach goal
    AVOIDING        = auto()   # Composite avoidance: back-up then arc-turn
    WANDERING       = auto()   # Drive forward with gentle arc (roomba-style)
    COMPLETE        = auto()   # All frontiers exhausted


# =============================================================================
#  Frontier data class
# =============================================================================

class Frontier:
    __slots__ = ("cx", "cy", "size", "score")

    def __init__(self, cx: float, cy: float, size: int) -> None:
        self.cx    = cx
        self.cy    = cy
        self.size  = size
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

        # Runtime state — start directly selecting frontiers (no INIT_SPIN)
        self.state: State                           = State.SELECT_FRONTIER
        self.map_data: Optional[OccupancyGrid]      = None
        self.costmap_data: Optional[OccupancyGrid]  = None
        self.latest_scan: Optional[LaserScan]       = None
        self._enabled: bool                         = True

        # Robot pose (map frame)
        self.rx = self.ry = self.ryaw = 0.0

        # Navigation bookkeeping
        self._goal_handle              = None
        self._nav_t0: Optional[float]  = None
        self._nav_done: bool           = False
        self._nav_ok:   bool           = False

        # Counters
        self.consec_fail:  int = 0
        self.total_fail:   int = 0
        self._no_frontier_streak: int = 0

        # Avoidance bookkeeping
        self._avoid_t0: Optional[float] = None
        self._avoid_phase: int          = 0   # 0=back-up, 1=arc-turn
        self._avoid_dir: float          = 1.0  # +1 or -1 (random arc direction)

        # Wandering bookkeeping
        self._wander_t0: Optional[float] = None
        self._wander_arc_dir: float      = 1.0

        # Visited frontier history (bounded deque)
        self._visited: Deque[Tuple[float, float]] = deque(maxlen=24)

        # Blacklist for unreachable goals: (x, y, timestamp)
        self._blacklist: Deque[Tuple[float, float, float]] = deque(maxlen=100)
        self._current_goal: Optional[Tuple[float, float]] = None

        # Failure-direction tracking: list of (bearing_rad, timestamp)
        self._failed_bearings: Deque[Tuple[float, float]] = deque(maxlen=100)

        # Ensure _state_complete executes exactly once
        self._completed: bool = False

        # Service readiness flags (updated by 1 Hz probe timer)
        self._svc_local_ok:  bool = False
        self._svc_global_ok: bool = False

        # Map data cache: avoid redundant numpy array conversions
        self._map_cache_header_stamp = None
        self._map_cache_array: Optional[np.ndarray] = None

        # Costmap data cache
        self._costmap_cache_header_stamp = None
        self._costmap_cache_array: Optional[np.ndarray] = None

        # Scan data cache
        self._scan_cache_seq: Optional[int] = None
        self._scan_cache_ranges: Optional[np.ndarray] = None
        self._scan_cache_angles: Optional[np.ndarray] = None

        # RNG for score perturbation & wander arc direction
        self._rng = np.random.default_rng()

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
        # Filtered scan publisher: only the front 120° arc, used by SLAM
        self._filtered_scan_pub = self.create_publisher(LaserScan, "/scan_filtered", 10)

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
            "║  v3.0  -  roomba-style, no spins, 120° lidar    ║\n"
            "║  Publish False to /explore/enable to pause.      ║\n"
            "╚══════════════════════════════════════════════════╝"
        )

    # -------------------------------------------------------------------------
    #  Parameters
    # -------------------------------------------------------------------------

    def _declare_params(self) -> None:
        self.declare_parameter("robot_frame",          "base_link")
        self.declare_parameter("map_frame",            "map")
        self.declare_parameter("min_frontier_size",    5)
        self.declare_parameter("obstacle_dist",        0.50)
        self.declare_parameter("nav_timeout",          35.0)    # s
        self.declare_parameter("backup_speed",        -0.18)    # m/s
        self.declare_parameter("backup_duration",      1.8)     # s
        self.declare_parameter("avoid_arc_speed",      0.10)    # m/s forward during arc
        self.declare_parameter("avoid_arc_yaw",        0.60)    # rad/s during arc-turn
        self.declare_parameter("avoid_arc_duration",   2.5)     # s
        self.declare_parameter("max_consec_fail",      4)
        self.declare_parameter("costmap_clear_every",  3)
        self.declare_parameter("complete_no_frontier", 8)
        self.declare_parameter("log_interval",        12.0)     # s
        self.declare_parameter("save_map_on_complete", True)
        self.declare_parameter("map_save_path",       "/tmp/leo_explored_map")
        self.declare_parameter("blacklist_radius",     2.0)     # m
        self.declare_parameter("blacklist_duration",   300.0)   # s
        # New v3.0 parameters
        self.declare_parameter("lidar_fov_deg",       120.0)    # front-facing FOV
        self.declare_parameter("wander_speed",         0.12)    # m/s
        self.declare_parameter("wander_arc_yaw",       0.20)    # rad/s gentle arc
        self.declare_parameter("wander_duration",      6.0)     # s per wander segment

    def _load_params(self) -> None:
        g = self.get_parameter
        self.p_robot_frame    = g("robot_frame").value
        self.p_map_frame      = g("map_frame").value
        self.p_min_frontier   = g("min_frontier_size").value
        self.p_obs_dist       = g("obstacle_dist").value
        self.p_nav_timeout    = g("nav_timeout").value
        self.p_backup_speed   = g("backup_speed").value
        self.p_backup_dur     = g("backup_duration").value
        self.p_arc_speed      = g("avoid_arc_speed").value
        self.p_arc_yaw        = g("avoid_arc_yaw").value
        self.p_arc_dur        = g("avoid_arc_duration").value
        self.p_max_consec     = g("max_consec_fail").value
        self.p_clear_every    = g("costmap_clear_every").value
        self.p_no_front_done  = g("complete_no_frontier").value
        self.p_log_interval   = g("log_interval").value
        self.p_save_map       = g("save_map_on_complete").value
        self.p_map_path       = g("map_save_path").value
        self.p_bl_radius      = g("blacklist_radius").value
        self.p_bl_duration    = g("blacklist_duration").value
        # v3.0
        self.p_lidar_half_angle = math.radians(g("lidar_fov_deg").value / 2.0)
        self.p_wander_speed     = g("wander_speed").value
        self.p_wander_arc_yaw   = g("wander_arc_yaw").value
        self.p_wander_dur       = g("wander_duration").value

    # -------------------------------------------------------------------------
    #  Callbacks
    # -------------------------------------------------------------------------

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.map_data = msg
        self._map_cache_header_stamp = None
        self._map_cache_array = None

    def _costmap_cb(self, msg: OccupancyGrid) -> None:
        self.costmap_data = msg
        self._costmap_cache_header_stamp = None
        self._costmap_cache_array = None

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self._scan_cache_seq = None
        # Publish filtered (front-arc only) scan for SLAM
        self._publish_filtered_scan(msg)

    def _enable_cb(self, msg: Bool) -> None:
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
    #  120° front-facing LiDAR filter
    # -------------------------------------------------------------------------

    def _publish_filtered_scan(self, scan: LaserScan) -> None:
        """
        Publish a copy of the scan with rays outside the front ±60° arc
        replaced by inf.  SLAM Toolbox subscribes to /scan_filtered instead
        of /scan, so it only sees the forward view.  This prevents rear
        laser data from creating conflicting scan matches when the robot
        turns around, which is the root cause of the ghost-wall phenomenon.
        """
        n = len(scan.ranges)
        if n == 0:
            return

        # Build angle array
        angles = (
            np.arange(n, dtype=np.float32) * scan.angle_increment
            + scan.angle_min
        )

        # Mask: keep only front arc
        keep = np.abs(angles) <= self.p_lidar_half_angle

        # Build filtered ranges
        ranges = np.array(scan.ranges, dtype=np.float32)
        filtered = np.where(keep, ranges, float("inf"))

        # Publish
        out = LaserScan()
        out.header = scan.header
        out.angle_min = scan.angle_min
        out.angle_max = scan.angle_max
        out.angle_increment = scan.angle_increment
        out.time_increment = scan.time_increment
        out.scan_time = scan.scan_time
        out.range_min = scan.range_min
        out.range_max = scan.range_max
        out.ranges = filtered.tolist()
        if scan.intensities:
            intensities = np.array(scan.intensities, dtype=np.float32)
            out.intensities = np.where(keep, intensities, 0.0).tolist()
        self._filtered_scan_pub.publish(out)

    # -------------------------------------------------------------------------
    #  Timing helper (use ROS clock for sim_time compatibility)
    # -------------------------------------------------------------------------

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _get_map_array(self) -> Optional[np.ndarray]:
        if self.map_data is None:
            return None
        stamp = self.map_data.header.stamp
        if (
            self._map_cache_array is not None
            and self._map_cache_header_stamp == stamp
        ):
            return self._map_cache_array
        w = self.map_data.info.width
        h = self.map_data.info.height
        self._map_cache_array = np.array(
            self.map_data.data, dtype=np.int8
        ).reshape((h, w))
        self._map_cache_header_stamp = stamp
        return self._map_cache_array

    def _get_costmap_array(self) -> Optional[np.ndarray]:
        if self.costmap_data is None:
            return None
        stamp = self.costmap_data.header.stamp
        if (
            self._costmap_cache_array is not None
            and self._costmap_cache_header_stamp == stamp
        ):
            return self._costmap_cache_array
        w = self.costmap_data.info.width
        h = self.costmap_data.info.height
        self._costmap_cache_array = np.array(
            self.costmap_data.data, dtype=np.int8
        ).reshape((h, w))
        self._costmap_cache_header_stamp = stamp
        return self._costmap_cache_array

    # -------------------------------------------------------------------------
    #  Service availability probe (1 Hz, non-blocking)
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

    def _obstacle_in_front(self) -> Tuple[bool, float]:
        """
        Check for obstacles in the front 120° arc only.
        Uses cached numpy arrays for performance.
        """
        scan = self.latest_scan
        if scan is None:
            return False, float("inf")

        scan_stamp = scan.header.stamp
        if self._scan_cache_seq != scan_stamp:
            self._scan_cache_ranges = np.array(scan.ranges, dtype=np.float32)
            self._scan_cache_angles = (
                np.arange(len(scan.ranges), dtype=np.float32)
                * scan.angle_increment
                + scan.angle_min
            )
            self._scan_cache_seq = scan_stamp

        ranges = self._scan_cache_ranges
        angles = self._scan_cache_angles
        valid = (
            np.isfinite(ranges)
            & (ranges > 0.01)
            & (np.abs(angles) <= self.p_lidar_half_angle)
        )
        if not np.any(valid):
            return False, float("inf")
        min_d = float(np.min(ranges[valid]))
        return min_d < self.p_obs_dist, min_d

    def _stop(self)  -> None: self._cmd_vel_pub.publish(Twist())

    def _drive(self, vx: float, wz: float = 0.0) -> None:
        t = Twist()
        t.linear.x  = vx
        t.angular.z = wz
        self._cmd_vel_pub.publish(t)

    # -------------------------------------------------------------------------
    #  Map statistics
    # -------------------------------------------------------------------------

    def _map_stats(self) -> Tuple[int, int, int, float]:
        raw = self._get_map_array()
        if raw is None:
            return 0, 0, 0, 0.0
        d    = raw.ravel()
        total = d.size
        shifted = d.astype(np.int16) + 1
        counts = np.bincount(shifted, minlength=102)
        unk  = int(counts[0])
        free = int(counts[1])
        occ  = int(counts[2:102].sum())
        pct  = (free + occ) / total * 100.0
        return free, occ, unk, pct

    def _explored_pct(self) -> float:
        return self._map_stats()[3]

    # -------------------------------------------------------------------------
    #  Frontier detection
    # -------------------------------------------------------------------------

    def _detect_frontiers(self) -> List[Frontier]:
        raw = self._get_map_array()
        if raw is None:
            return []

        h, w = raw.shape
        res = self.map_data.info.resolution
        ox  = self.map_data.info.origin.position.x
        oy  = self.map_data.info.origin.position.y

        free    = raw == 0
        unknown = raw == -1

        # Pad-based 4-connected dilation (no edge-wrap artefacts)
        padded = np.pad(unknown, 1, mode="constant", constant_values=False)
        unk_d  = (
            padded[:-2, 1:-1]
            | padded[2:,  1:-1]
            | padded[1:-1, :-2]
            | padded[1:-1, 2:]
        )

        frontier_mask = free & unk_d

        # BFS 8-connected clustering
        labeled    = np.zeros((h, w), dtype=np.int32)
        cluster_id = 0
        clusters: dict = {}

        ys, xs = np.where(frontier_mask)
        for i in range(len(ys)):
            sy, sx = int(ys[i]), int(xs[i])
            if labeled[sy, sx] != 0:
                continue
            cluster_id += 1
            cells: List[Tuple[int, int]] = []
            clusters[cluster_id] = cells
            q: deque = deque()
            q.append((sy, sx))
            labeled[sy, sx] = cluster_id
            while q:
                cy, cx = q.popleft()
                cells.append((cx, cy))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        ny, nx = cy + dy, cx + dx
                        if (
                            0 <= ny < h
                            and 0 <= nx < w
                            and frontier_mask[ny, nx]
                            and labeled[ny, nx] == 0
                        ):
                            labeled[ny, nx] = cluster_id
                            q.append((ny, nx))

        # Convert to world coordinates
        candidates: List[Frontier] = []
        for cid, cells in clusters.items():
            n_cells = len(cells)
            if n_cells < self.p_min_frontier:
                continue
            cells_arr = np.array(cells, dtype=np.float32)
            mean_col = cells_arr[:, 0].mean()
            mean_row = cells_arr[:, 1].mean()
            wx = mean_col * res + ox
            wy = mean_row * res + oy
            candidates.append(Frontier(wx, wy, n_cells))

        # Batch costmap inflation filter
        frontiers = self._filter_frontiers_by_costmap(candidates)
        return frontiers

    def _filter_frontiers_by_costmap(
        self, frontiers: List[Frontier]
    ) -> List[Frontier]:
        if not frontiers:
            return []

        cm = self.costmap_data
        if cm is None:
            return frontiers

        cm_arr = self._get_costmap_array()
        if cm_arr is None:
            return frontiers

        res = cm.info.resolution
        ox  = cm.info.origin.position.x
        oy  = cm.info.origin.position.y
        h, w = cm_arr.shape

        wx = np.array([f.cx for f in frontiers], dtype=np.float32)
        wy = np.array([f.cy for f in frontiers], dtype=np.float32)

        cols = ((wx - ox) / res).astype(np.int32)
        rows = ((wy - oy) / res).astype(np.int32)

        in_bounds = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
        in_inflation = np.zeros(len(frontiers), dtype=bool)

        if np.any(in_bounds):
            ib_rows = rows[in_bounds]
            ib_cols = cols[in_bounds]
            costs = cm_arr[ib_rows, ib_cols].astype(np.int16)
            unsigned_costs = costs + np.int16(256) * (costs < 0)
            in_inflation[in_bounds] = (
                (unsigned_costs >= COSTMAP_LETHAL_THRESH)
                & (unsigned_costs < 255)
            )

        return [f for f, blocked in zip(frontiers, in_inflation) if not blocked]

    # -------------------------------------------------------------------------
    #  Frontier scoring
    # -------------------------------------------------------------------------

    def _score_frontiers(
        self,
        frontiers: List[Frontier],
        rx: float, ry: float, ryaw: float,
    ) -> List[Frontier]:
        """
        score = 0.30 * info_gain
              + 0.25 * dist_score
              + 0.35 * dir_score       ← increased from 0.25 to favour
                                          frontiers in front of the robot
              - visit_penalty
              - fail_dir_penalty

        Frontiers closer than 0.5 m are filtered out.
        """
        if not frontiers:
            return []

        n = len(frontiers)
        fx = np.array([f.cx for f in frontiers], dtype=np.float32)
        fy = np.array([f.cy for f in frontiers], dtype=np.float32)
        sizes = np.array([f.size for f in frontiers], dtype=np.float32)

        # Vectorized distance calculation
        dists = np.hypot(fx - rx, fy - ry)

        # Filter out frontiers too close (<0.5 m)
        valid_mask = dists >= 0.5

        # Filter out frontiers near blacklisted (unreachable) goals
        now = self._now_sec()
        while self._blacklist and (now - self._blacklist[0][2]) > self.p_bl_duration:
            self._blacklist.popleft()
        adaptive_bl_radius = self.p_bl_radius * (1.0 + 0.3 * self.consec_fail)
        if self._blacklist:
            bl_arr = np.array(
                [(x, y) for x, y, _ in self._blacklist], dtype=np.float32
            )
            dists_to_bl = np.sqrt(
                (fx[:, np.newaxis] - bl_arr[:, 0]) ** 2
                + (fy[:, np.newaxis] - bl_arr[:, 1]) ** 2
            )
            valid_mask &= ~np.any(dists_to_bl < adaptive_bl_radius, axis=1)

        # Info gain (normalised cluster size)
        info_scores = np.minimum(1.0, sizes / 60.0)

        # Distance score: prefer 1.0-4.0 m range
        dist_scores = np.where(
            dists <= 4.0,
            1.0 - np.abs(dists - 2.0) / 4.0,
            np.maximum(0.0, 1.0 - (dists - 4.0) / 6.0)
        )

        # Direction score: prefer frontiers ahead of robot (v3.0: weight ↑)
        angles_to = np.arctan2(fy - ry, fx - rx)
        angle_diffs = np.abs(np.arctan2(
            np.sin(angles_to - ryaw),
            np.cos(angles_to - ryaw),
        ))
        dir_scores = np.maximum(0.0, 1.0 - angle_diffs / np.pi)

        # Revisit penalty
        visit_pens = np.zeros(n, dtype=np.float32)
        if self._visited:
            visited_arr = np.array(list(self._visited), dtype=np.float32)
            dists_to_visited = np.sqrt(
                (fx[:, np.newaxis] - visited_arr[:, 0]) ** 2 +
                (fy[:, np.newaxis] - visited_arr[:, 1]) ** 2
            )
            visit_pens = np.where(np.any(dists_to_visited < 1.0, axis=1), 0.55, 0.0)

        # Failure-direction penalty
        while (self._failed_bearings
               and (now - self._failed_bearings[0][1]) > self.p_bl_duration):
            self._failed_bearings.popleft()
        fail_dir_pens = np.zeros(n, dtype=np.float32)
        if self._failed_bearings:
            fb_arr = np.array(
                [b for b, _ in self._failed_bearings], dtype=np.float32
            )
            bearing_diffs = np.abs(np.arctan2(
                np.sin(angles_to[:, np.newaxis] - fb_arr[np.newaxis, :]),
                np.cos(angles_to[:, np.newaxis] - fb_arr[np.newaxis, :]),
            ))
            fail_dir_pens = np.where(
                np.any(bearing_diffs < math.radians(30.0), axis=1),
                0.35, 0.0
            )

        # v3.0: rebalanced scoring weights — direction heavily favoured
        scores = (
            0.30 * info_scores
            + 0.25 * dist_scores
            + 0.35 * dir_scores
            - visit_pens
            - fail_dir_pens
        )

        # Random perturbation after 2+ consecutive failures
        if self.consec_fail >= 2:
            scores += self._rng.uniform(-0.15, 0.15, size=n).astype(np.float32)

        # Build result list with only valid frontiers
        result: List[Frontier] = []
        for i, f in enumerate(frontiers):
            if valid_mask[i]:
                f.score = float(scores[i])
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

        # v3.0: Set goal orientation to face TOWARD the goal from current
        # position, so Nav2 won't command a large rotation on arrival.
        dx = fx - self.rx
        dy = fy - self.ry
        goal_yaw = math.atan2(dy, dx)
        goal.pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(goal_yaw / 2.0)

        self._nav_done = False
        self._nav_ok   = False
        self._nav_t0   = self._now_sec()
        self._current_goal = (fx, fy)

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

    def _blacklist_current_goal(self) -> None:
        if self._current_goal is None:
            return
        gx, gy = self._current_goal
        now = self._now_sec()
        self._blacklist.append((gx, gy, now))
        pose = self._robot_in_map()
        if pose is not None:
            cur_rx, cur_ry, _ = pose
        else:
            cur_rx, cur_ry = self.rx, self.ry
        bearing = math.atan2(gy - cur_ry, gx - cur_rx)
        self._failed_bearings.append((bearing, now))
        self.get_logger().info(
            f"Blacklisted unreachable goal ({gx:.2f}, {gy:.2f})"
        )

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
            r_sz      = 0.08 + 0.04 * min(1.0, f.size / 40.0)
            m.scale.x = r_sz * 2
            m.scale.y = r_sz * 2
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
        {
            State.SELECT_FRONTIER: self._state_select,
            State.NAVIGATING:      self._state_navigating,
            State.AVOIDING:        self._state_avoiding,
            State.WANDERING:       self._state_wandering,
            State.COMPLETE:        self._state_complete,
        }[self.state](now)

    # -------------------------------------------------------------------------
    #  State: SELECT_FRONTIER
    # -------------------------------------------------------------------------

    def _state_select(self, now: float) -> None:
        if self.map_data is None:
            # No map yet — wander forward to generate initial scan data
            self._drive(self.p_wander_speed, 0.0)
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
            self._no_frontier_streak += 1
            self.get_logger().warn(
                f"No frontiers found (streak={self._no_frontier_streak})"
            )
            if self._no_frontier_streak >= self.p_no_front_done:
                self.get_logger().info("No frontiers left - exploration complete!")
                self.state = State.COMPLETE
            else:
                # Wander forward instead of spinning
                self.state = State.WANDERING
            return

        self._no_frontier_streak = 0

        if self.consec_fail >= self.p_max_consec:
            self.get_logger().warn("Too many consecutive failures - wandering")
            self.consec_fail = 0
            self.state = State.WANDERING
            return

        scored = self._score_frontiers(raw_frontiers, rx, ry, ryaw)
        self._publish_frontiers(scored[:20])

        # All frontiers too close (filtered out by _score_frontiers)
        if not scored:
            self.get_logger().warn(
                "All frontiers too close — wandering forward"
            )
            self.state = State.WANDERING
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
        # Emergency obstacle avoidance (threshold = 65% of configured distance)
        obs, dist = self._obstacle_in_front()
        if obs and dist < self.p_obs_dist * 0.65:
            self.get_logger().warn(
                f"Obstacle at {dist:.2f} m - emergency avoidance"
            )
            self._cancel_nav()
            self._avoid_t0    = now
            self._avoid_phase = 0
            # Random arc direction for avoidance
            self._avoid_dir = 1.0 if self._rng.random() > 0.5 else -1.0
            self.state = State.AVOIDING
            return

        # Navigation result arrived via callback
        if self._nav_done:
            if self._nav_ok:
                self.consec_fail = 0
                self._current_goal = None
            else:
                self.consec_fail += 1
                self.total_fail  += 1
                self._blacklist_current_goal()
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
            self._blacklist_current_goal()
            self._nav_done    = False
            self.state = State.SELECT_FRONTIER

    # -------------------------------------------------------------------------
    #  State: AVOIDING  (v3.0: backup + arc-turn, no pure spin)
    # -------------------------------------------------------------------------

    def _state_avoiding(self, now: float) -> None:
        if self._avoid_phase == 0:
            # Phase 0: back up
            elapsed = now - (self._avoid_t0 or now)
            if elapsed < self.p_backup_dur:
                self._drive(self.p_backup_speed)
            else:
                # Transition to arc-turn
                self._avoid_phase = 1
                self._avoid_t0    = now
        else:
            # Phase 1: arc-turn (drive forward with angular velocity)
            elapsed = now - (self._avoid_t0 or now)
            if elapsed < self.p_arc_dur:
                self._drive(self.p_arc_speed, self._avoid_dir * self.p_arc_yaw)
            else:
                self._stop()
                self.get_logger().info("Avoidance manoeuvre complete")
                self.consec_fail += 1
                self._blacklist_current_goal()
                self.state = State.SELECT_FRONTIER

    # -------------------------------------------------------------------------
    #  State: WANDERING  (v3.0: roomba-style forward drive with gentle arc)
    # -------------------------------------------------------------------------

    def _state_wandering(self, now: float) -> None:
        if self._wander_t0 is None:
            self._wander_t0 = now
            # Random gentle arc direction
            self._wander_arc_dir = 1.0 if self._rng.random() > 0.5 else -1.0
            self.get_logger().info("Wandering forward (roomba-style)...")

        # Check for obstacle
        obs, dist = self._obstacle_in_front()
        if obs and dist < self.p_obs_dist:
            self.get_logger().info(
                f"Obstacle during wander at {dist:.2f} m - avoiding"
            )
            self._wander_t0 = None
            self._avoid_t0    = now
            self._avoid_phase = 0
            self._avoid_dir = 1.0 if self._rng.random() > 0.5 else -1.0
            self.state = State.AVOIDING
            return

        elapsed = now - self._wander_t0
        if elapsed < self.p_wander_dur:
            self._drive(self.p_wander_speed,
                        self._wander_arc_dir * self.p_wander_arc_yaw)
        else:
            self._stop()
            self._wander_t0 = None
            self.get_logger().info("Wander segment done — re-selecting frontier")
            self.state = State.SELECT_FRONTIER

    # -------------------------------------------------------------------------
    #  State: COMPLETE (exactly-once guard)
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
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
