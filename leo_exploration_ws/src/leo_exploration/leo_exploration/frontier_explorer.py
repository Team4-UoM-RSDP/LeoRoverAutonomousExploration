#!/usr/bin/env python3
"""
Leo Rover Frontier-Based Autonomous Exploration Node
ROS2 Jazzy  |  v2.4

Changelog v2.4 (adaptive blacklisting & scoring rebalance):
  Bug 7  - Adaptive blacklist radius: base radius (2.0 m) scales up with
            consecutive failures via factor (1 + 0.3 * consec_fail), so the
            robot avoids wider areas around repeatedly unreachable goals.
  Bug 8  - Failure-direction penalty: bearings toward failed goals are
            tracked; frontiers within 30° of a recent failure bearing are
            penalised, preventing the robot from circling in the same
            direction.
  Bug 9  - Scoring weight rebalance: info_gain 0.45→0.35, distance
            0.35→0.30, direction 0.15→0.25.  Increased direction weight
            steers the robot toward unexplored headings rather than fixating
            on high-info-gain frontiers that may be unreachable.
  Bug 10 - Stronger revisit penalty: 0.40 within 0.6 m → 0.55 within
            1.0 m.  Discourages circling over already-visited territory.
  Bug 11 - Random score perturbation (±0.15) after 2+ consecutive failures
            breaks deterministic re-selection of the same frontier.
  Bug 12 - Blacklist capacity raised 50→100, duration 120→300 s, default
            radius 1.0→2.0 m for more effective exclusion of stuck regions.

Changelog v2.3 (frontier blacklisting):
  Bug 6  - Unreachable goal blacklist: when Nav2 fails to plan or navigate
            to a frontier, its coordinates are added to a time-decaying
            blacklist. Frontiers within blacklist_radius (default 1.0 m) of
            any blacklisted point are excluded from selection, preventing
            the robot from repeatedly targeting the same unreachable goal.

Changelog v2.2 (performance optimizations round 2):
  Perf 5 - Scan array caching: _obstacle_in_sector() caches numpy arrays
            (ranges + angles) to avoid O(n) conversion every 5 Hz tick.
  Perf 6 - Single-pass map stats: _map_stats() uses np.bincount instead of
            three separate np.sum calls, reducing passes over the array.
  Perf 7 - Costmap array caching: _get_costmap_array() caches numpy
            conversion, mirroring the map cache strategy.
  Perf 8 - Batched inflation filter: _filter_frontiers_by_costmap() checks
            all frontier centroids in one vectorized numpy operation instead
            of per-frontier Python calls.
  Perf 9 - Removed dead visited-list trimming code in _score_frontiers()
            (deque maxlen=24 makes the len>50 check unreachable).

Changelog v2.1 (performance optimizations):
  Perf 1 - Map array caching: _get_map_array() caches numpy conversion,
            avoiding O(w*h) conversion on every control loop iteration.
  Perf 2 - Vectorized frontier scoring: _score_frontiers() uses numpy
            for batch distance, angle, and penalty calculations.
  Perf 3 - Vectorized centroid calculation: uses numpy array operations
            instead of Python sum() for cluster centroid computation.
  Perf 4 - Optimized BFS iteration: direct numpy array indexing instead
            of converting to Python lists with .tolist().

Changelog v2.0 (analysis_report.md fixes applied):
  Bug 1  - Extract _map_stats() helper: _log_progress / _explored_pct share
            one code path (DRY, single source of truth).
  Bug 2  - np.roll replaced with np.pad dilation: no edge-wrap artefacts that
            produced spurious frontier detections along map borders.
  Bug 3  - _clear_costmaps: blocking wait_for_service removed from control
            loop; service readiness tracked by a separate 1 Hz timer.
  Bug 4  - _state_complete guarded by _completed flag -> prints exactly once.
  Bug 5  - All timing uses _now_sec() (ROS clock) -> sim_time-compatible;
            time.monotonic() fully removed.
  P1     - Frontier reachability filter: centroids inside costmap
            inflation/lethal zone (cost >= COSTMAP_LETHAL_THRESH) discarded.
  P1     - Subscribe /global_costmap/costmap for the inflation filter.
  P2     - Map auto-saved on COMPLETE via nav2_map_server map_saver_cli.
  P2     - Stop/resume via Bool topic /explore/enable.
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
    INIT_SPIN       = auto()   # Initial 360 spin to seed SLAM map
    SELECT_FRONTIER = auto()   # Pick best frontier and dispatch Nav2 goal
    NAVIGATING      = auto()   # Waiting for Nav2 to reach goal
    AVOIDING        = auto()   # Composite avoidance: back-up then spin
    RECOVERING      = auto()   # Slow spin to expose new frontiers
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

        # Runtime state
        self.state: State                           = State.INIT_SPIN
        self.map_data: Optional[OccupancyGrid]      = None
        self.costmap_data: Optional[OccupancyGrid]  = None   # P1
        self.latest_scan: Optional[LaserScan]       = None
        self._enabled: bool                         = True   # P2 pause/resume

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

        # Blacklist for unreachable goals: (x, y, timestamp)
        self._blacklist: Deque[Tuple[float, float, float]] = deque(maxlen=100)
        self._current_goal: Optional[Tuple[float, float]] = None

        # Failure-direction tracking: list of (bearing_rad, timestamp)
        self._failed_bearings: Deque[Tuple[float, float]] = deque(maxlen=100)

        # Bug 4: ensure _state_complete executes exactly once
        self._completed: bool = False

        # Bug 3: service readiness flags (updated by 1 Hz probe timer)
        self._svc_local_ok:  bool = False
        self._svc_global_ok: bool = False

        # Map data cache: avoid redundant numpy array conversions
        self._map_cache_header_stamp = None
        self._map_cache_array: Optional[np.ndarray] = None

        # Costmap data cache (Perf 7): same strategy as map cache
        self._costmap_cache_header_stamp = None
        self._costmap_cache_array: Optional[np.ndarray] = None

        # Scan data cache (Perf 5): avoid rebuilding numpy arrays every tick
        self._scan_cache_seq: Optional[int] = None
        self._scan_cache_ranges: Optional[np.ndarray] = None
        self._scan_cache_angles: Optional[np.ndarray] = None

        # RNG for score perturbation (Bug 11): reuse to avoid repeated instantiation
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
                                 self._costmap_cb, tl_qos)   # P1
        self.create_subscription(LaserScan, "/scan",
                                 self._scan_cb, 10)
        self.create_subscription(Bool, "/explore/enable",
                                 self._enable_cb, 10)         # P2

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
        self._svc_timer  = self.create_timer(1.0,                  self._probe_services)  # Bug 3

        self.get_logger().info(
            "\n"
            "╔══════════════════════════════════════════════╗\n"
            "║  Leo Rover Frontier Explorer  (ROS2 Jazzy)   ║\n"
            "║  v2.0  -  starting initial 360 spin...       ║\n"
            "║  Publish False to /explore/enable to pause.  ║\n"
            "╚══════════════════════════════════════════════╝"
        )

    # -------------------------------------------------------------------------
    #  Parameters
    # -------------------------------------------------------------------------

    def _declare_params(self) -> None:
        self.declare_parameter("robot_frame",          "base_link")
        self.declare_parameter("map_frame",            "map")
        self.declare_parameter("min_frontier_size",    5)
        self.declare_parameter("obstacle_dist",        0.50)
        self.declare_parameter("obstacle_half_angle",  50.0)    # degrees
        self.declare_parameter("nav_timeout",          35.0)    # s
        self.declare_parameter("spin_speed",           0.55)    # rad/s
        self.declare_parameter("spin_duration",        12.5)    # s
        self.declare_parameter("backup_speed",        -0.18)    # m/s
        self.declare_parameter("backup_duration",      1.8)     # s
        self.declare_parameter("avoid_spin_duration",  2.5)     # s
        self.declare_parameter("recov_spin_duration",  7.0)     # s
        self.declare_parameter("max_consec_fail",      4)
        self.declare_parameter("costmap_clear_every",  3)
        self.declare_parameter("complete_no_frontier", 8)
        self.declare_parameter("log_interval",        12.0)     # s
        self.declare_parameter("save_map_on_complete", True)    # P2
        self.declare_parameter("map_save_path",       "/tmp/leo_explored_map")  # P2
        self.declare_parameter("blacklist_radius",     2.0)     # m
        self.declare_parameter("blacklist_duration",   300.0)   # s

    def _load_params(self) -> None:
        g = self.get_parameter
        self.p_robot_frame    = g("robot_frame").value
        self.p_map_frame      = g("map_frame").value
        self.p_min_frontier   = g("min_frontier_size").value
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
        self.p_bl_radius      = g("blacklist_radius").value
        self.p_bl_duration    = g("blacklist_duration").value

    # -------------------------------------------------------------------------
    #  Callbacks
    # -------------------------------------------------------------------------

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.map_data = msg
        # Invalidate cache when new map arrives
        self._map_cache_header_stamp = None
        self._map_cache_array = None

    def _costmap_cb(self, msg: OccupancyGrid) -> None:
        self.costmap_data = msg
        # Invalidate costmap cache when new data arrives (Perf 7)
        self._costmap_cache_header_stamp = None
        self._costmap_cache_array = None

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        # Invalidate scan cache when new data arrives (Perf 5)
        self._scan_cache_seq = None

    def _enable_cb(self, msg: Bool) -> None:
        """P2: publish std_msgs/Bool False to /explore/enable to pause."""
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
    #  Timing helper  (Bug 5: use ROS clock, not time.monotonic)
    # -------------------------------------------------------------------------

    def _now_sec(self) -> float:
        """Seconds from ROS clock. Works with use_sim_time:=true."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _get_map_array(self) -> Optional[np.ndarray]:
        """
        Return cached numpy array of map data, converting only when map changes.
        Avoids O(w*h) array conversion on every call to _detect_frontiers
        and _map_stats when the map hasn't been updated.
        """
        if self.map_data is None:
            return None

        stamp = self.map_data.header.stamp
        if (
            self._map_cache_array is not None
            and self._map_cache_header_stamp == stamp
        ):
            return self._map_cache_array

        # Map changed - rebuild cache
        w = self.map_data.info.width
        h = self.map_data.info.height
        self._map_cache_array = np.array(
            self.map_data.data, dtype=np.int8
        ).reshape((h, w))
        self._map_cache_header_stamp = stamp
        return self._map_cache_array

    def _get_costmap_array(self) -> Optional[np.ndarray]:
        """
        Perf 7: Return cached numpy array of costmap data.
        Same caching strategy as _get_map_array().
        """
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
    #  Service availability probe  (Bug 3: 1 Hz, non-blocking)
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
        """
        Perf 5: Numpy-vectorised forward obstacle check with cached arrays.
        Caches the ranges/angles numpy conversion to avoid O(n) allocation
        on every 5 Hz control-loop tick when scan data hasn't changed.
        """
        scan = self.latest_scan
        if scan is None:
            return False, float("inf")

        # Cache scan numpy arrays, keyed by header stamp
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
            & (np.abs(angles) <= self.p_obs_half_angle)
        )
        if not np.any(valid):
            return False, float("inf")
        min_d = float(np.min(ranges[valid]))
        return min_d < self.p_obs_dist, min_d

    def _stop(self)  -> None: self._cmd_vel_pub.publish(Twist())

    def _spin(self, speed: Optional[float] = None) -> None:
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
    #  Map statistics  (Bug 1: single source of truth for progress data)
    # -------------------------------------------------------------------------

    def _map_stats(self) -> Tuple[int, int, int, float]:
        """
        Returns (free_cells, occupied_cells, unknown_cells, explored_pct).
        Uses int8 dtype: unknown=-1, free=0, occupied=1..100.
        Both _log_progress() and _explored_pct() delegate here (Bug 1 fix).
        Uses cached array to avoid redundant numpy conversions.

        Perf 6: Uses np.bincount for single-pass counting instead of three
        separate np.sum operations.
        """
        raw = self._get_map_array()
        if raw is None:
            return 0, 0, 0, 0.0
        d    = raw.ravel()
        total = d.size
        # Shift values: int8 -1→0, 0→1, 1→2, ..., 100→101
        # so all values are non-negative for bincount
        shifted = d.astype(np.int16) + 1
        counts = np.bincount(shifted, minlength=102)
        unk  = int(counts[0])          # original -1
        free = int(counts[1])          # original  0
        occ  = int(counts[2:102].sum())  # original 1..100
        pct  = (free + occ) / total * 100.0
        return free, occ, unk, pct

    def _explored_pct(self) -> float:
        return self._map_stats()[3]

    # -------------------------------------------------------------------------
    #  Frontier detection
    # -------------------------------------------------------------------------

    def _detect_frontiers(self) -> List[Frontier]:
        """
        Detect free cells adjacent to unknown cells, cluster them, and apply
        the costmap inflation filter (P1).

        Bug 2 fix: dilation uses np.pad instead of np.roll. np.roll wraps the
        last row/column around to the opposite edge, creating phantom frontier
        detections at map borders even after manual edge-clearing. np.pad with
        constant_values=False (zero) avoids this entirely.
        """
        raw = self._get_map_array()
        if raw is None:
            return []

        h, w = raw.shape
        res = self.map_data.info.resolution
        ox  = self.map_data.info.origin.position.x
        oy  = self.map_data.info.origin.position.y

        free    = raw == 0
        unknown = raw == -1

        # Bug 2 fix: pad-based 4-connected dilation
        padded = np.pad(unknown, 1, mode="constant", constant_values=False)
        unk_d  = (
            padded[:-2, 1:-1]    # row above
            | padded[2:,  1:-1]  # row below
            | padded[1:-1, :-2]  # col left
            | padded[1:-1, 2:]   # col right
        )

        frontier_mask = free & unk_d

        # BFS 8-connected clustering
        labeled    = np.zeros((h, w), dtype=np.int32)
        cluster_id = 0
        clusters: dict = {}

        ys, xs = np.where(frontier_mask)
        # Iterate directly over numpy arrays - avoid Python list conversion overhead
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
                cells.append((cx, cy))          # (col, row)
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
            # Vectorized centroid calculation using numpy
            cells_arr = np.array(cells, dtype=np.float32)
            mean_col = cells_arr[:, 0].mean()
            mean_row = cells_arr[:, 1].mean()
            wx = mean_col * res + ox
            wy = mean_row * res + oy
            candidates.append(Frontier(wx, wy, n_cells))

        # Perf 8: Batch costmap inflation filter
        frontiers = self._filter_frontiers_by_costmap(candidates)
        return frontiers

    def _filter_frontiers_by_costmap(
        self, frontiers: List[Frontier]
    ) -> List[Frontier]:
        """
        Perf 8: Batch-check all frontier centroids against the costmap
        inflation zone in one vectorized numpy operation, replacing the
        per-frontier _frontier_in_inflation() Python loop.
        """
        if not frontiers:
            return []

        cm = self.costmap_data
        if cm is None:
            return frontiers  # costmap not yet available: allow all through

        cm_arr = self._get_costmap_array()
        if cm_arr is None:
            return frontiers

        res = cm.info.resolution
        ox  = cm.info.origin.position.x
        oy  = cm.info.origin.position.y
        h, w = cm_arr.shape

        # Build coordinate arrays for all frontiers at once
        wx = np.array([f.cx for f in frontiers], dtype=np.float32)
        wy = np.array([f.cy for f in frontiers], dtype=np.float32)

        cols = ((wx - ox) / res).astype(np.int32)
        rows = ((wy - oy) / res).astype(np.int32)

        # Mask: inside costmap bounds
        in_bounds = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)

        # For out-of-bounds frontiers, allow through (not in inflation)
        in_inflation = np.zeros(len(frontiers), dtype=bool)

        if np.any(in_bounds):
            ib_rows = rows[in_bounds]
            ib_cols = cols[in_bounds]
            costs = cm_arr[ib_rows, ib_cols].astype(np.int16)
            # The costmap array uses int8 dtype, so ROS values 128-255
            # wrap to negative in int8. Cast to int16 to recover the
            # original unsigned semantics for threshold comparison.
            # nav2 costmap: 0=free, 1-252=inflated, 253=inscribed,
            # 254=lethal, 255=unknown.  We filter >= COSTMAP_LETHAL_THRESH.
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
        score = 0.35 * info_gain
              + 0.30 * dist_score
              + 0.25 * dir_score
              - visit_penalty
              - fail_dir_penalty

        Frontiers closer than 0.5m are FILTERED OUT entirely — sending
        a goal that close causes the controller to instantly report
        'Reached the goal' without actually moving.
        """
        if not frontiers:
            return []

        # Perf 9: Removed dead visited-list trimming code. The _visited
        # deque has maxlen=24, so len() > 50 was unreachable.

        # Extract frontier coordinates as numpy arrays for vectorized computation
        n = len(frontiers)
        fx = np.array([f.cx for f in frontiers], dtype=np.float32)
        fy = np.array([f.cy for f in frontiers], dtype=np.float32)
        sizes = np.array([f.size for f in frontiers], dtype=np.float32)

        # Vectorized distance calculation
        dists = np.hypot(fx - rx, fy - ry)

        # Filter out frontiers too close (< 0.5m)
        valid_mask = dists >= 0.5

        # Filter out frontiers near blacklisted (unreachable) goals
        # Bug 7: adaptive blacklist radius scales with consecutive failures
        now = self._now_sec()
        # Prune expired blacklist entries
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

        # Vectorized information gain (normalised cluster size)
        info_scores = np.minimum(1.0, sizes / 60.0)

        # Vectorized distance score: prefer 1.0-4.0 m range
        dist_scores = np.where(
            dists <= 4.0,
            1.0 - np.abs(dists - 2.0) / 4.0,
            np.maximum(0.0, 1.0 - (dists - 4.0) / 6.0)
        )

        # Vectorized direction score: prefer frontiers ahead of robot
        angles_to = np.arctan2(fy - ry, fx - rx)
        angle_diffs = np.abs(np.arctan2(
            np.sin(angles_to - ryaw),
            np.cos(angles_to - ryaw),
        ))
        dir_scores = np.maximum(0.0, 1.0 - angle_diffs / np.pi)

        # Vectorized revisit penalty computation
        # Bug 10: stronger penalty 0.55 within 1.0 m
        visit_pens = np.zeros(n, dtype=np.float32)
        if self._visited:
            visited_arr = np.array(list(self._visited), dtype=np.float32)
            # Compute distances from each frontier to all visited points
            # Shape: (n_frontiers, n_visited)
            dists_to_visited = np.sqrt(
                (fx[:, np.newaxis] - visited_arr[:, 0]) ** 2 +
                (fy[:, np.newaxis] - visited_arr[:, 1]) ** 2
            )
            # Apply penalty if any visited point is within 1.0m
            visit_pens = np.where(np.any(dists_to_visited < 1.0, axis=1), 0.55, 0.0)

        # Bug 8: failure-direction penalty
        # Prune expired failure bearings (same duration as blacklist)
        while (self._failed_bearings
               and (now - self._failed_bearings[0][1]) > self.p_bl_duration):
            self._failed_bearings.popleft()
        fail_dir_pens = np.zeros(n, dtype=np.float32)
        if self._failed_bearings:
            fb_arr = np.array(
                [b for b, _ in self._failed_bearings], dtype=np.float32
            )
            # Angular difference between each frontier bearing and each
            # failed bearing — shape (n_frontiers, n_failed_bearings)
            bearing_diffs = np.abs(np.arctan2(
                np.sin(angles_to[:, np.newaxis] - fb_arr[np.newaxis, :]),
                np.cos(angles_to[:, np.newaxis] - fb_arr[np.newaxis, :]),
            ))
            # Penalise if any failed bearing is within 30°
            fail_dir_pens = np.where(
                np.any(bearing_diffs < math.radians(30.0), axis=1),
                0.35, 0.0
            )

        # Bug 9: rebalanced scoring weights
        scores = (
            0.35 * info_scores
            + 0.30 * dist_scores
            + 0.25 * dir_scores
            - visit_pens
            - fail_dir_pens
        )

        # Bug 11: random perturbation after 2+ consecutive failures
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
        goal.pose.pose.orientation.w = 1.0

        self._nav_done = False
        self._nav_ok   = False
        self._nav_t0   = self._now_sec()   # Bug 5: ROS clock
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
        """
        Bug 3 fix: never blocks the control loop.
        _probe_services() updates _svc_*_ok flags at 1 Hz without blocking.
        """
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
        """Add the current navigation goal to the blacklist so that nearby
        frontiers are excluded from future selection until the entry expires.
        Also record the bearing from the robot to the failed goal so that
        frontiers in the same direction receive a scoring penalty (Bug 8)."""
        if self._current_goal is None:
            return
        gx, gy = self._current_goal
        now = self._now_sec()
        self._blacklist.append((gx, gy, now))
        # Bug 8: record failure bearing for direction penalty
        # Use latest TF pose for accurate bearing calculation
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
    #  Map save  (P2)
    # -------------------------------------------------------------------------

    def _save_map(self) -> None:
        """P2: save the final map via nav2_map_server map_saver_cli."""
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
    #  Logging  (Bug 1: delegates to _map_stats)
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
            return    # P2: paused

        now = self._now_sec()   # Bug 5: ROS clock
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
            self._spin()
        else:
            self._stop()
            self._spin_t0 = None
            self.get_logger().info("Initial spin done - starting exploration")
            self.state = State.SELECT_FRONTIER

    # -------------------------------------------------------------------------
    #  State: SELECT_FRONTIER
    # -------------------------------------------------------------------------

    def _state_select(self, now: float) -> None:
        if self.map_data is None:
            self.get_logger().warn("Waiting for /map...")
            return

        # Periodic costmap clear on consecutive failures (non-blocking, Bug 3)
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
                self._current_goal = None
            else:
                self.consec_fail += 1
                self.total_fail  += 1
                self._blacklist_current_goal()
            self._nav_done = False
            self.state = State.SELECT_FRONTIER
            return

        # Timeout guard (uses ROS clock via _nav_t0 set in _send_goal)
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
    #  State: AVOIDING
    # -------------------------------------------------------------------------

    def _state_avoiding(self, now: float) -> None:
        if self._avoid_phase == 0:
            # Phase 0: back up
            elapsed = now - (self._avoid_t0 or now)
            if elapsed < self.p_backup_dur:
                self._drive(self.p_backup_speed)
            else:
                # Transition to spin; reset t0 BEFORE next tick (Bug 6 from prev)
                self._avoid_phase = 1
                self._avoid_t0    = now
                self._spin(self.p_spin_speed * 1.4)   # start immediately
        else:
            # Phase 1: spin to clear obstacle; elapsed measured from new t0
            elapsed = now - (self._avoid_t0 or now)
            if elapsed < self.p_avoid_spin_dur:
                self._spin(self.p_spin_speed * 1.4)
            else:
                self._stop()
                self.get_logger().info("Avoidance manoeuvre complete")
                self.consec_fail += 1
                self._blacklist_current_goal()
                self.state = State.SELECT_FRONTIER

    # -------------------------------------------------------------------------
    #  State: RECOVERING
    # -------------------------------------------------------------------------

    def _state_recovering(self, now: float) -> None:
        if self._recov_t0 is None:
            self._recov_t0 = now
            self.get_logger().info("Recovery spin...")

        if now - self._recov_t0 < self.p_recov_spin_dur:
            self._spin()
        else:
            self._stop()
            self._recov_t0 = None
            self.get_logger().info("Recovery spin done")
            self.state = State.SELECT_FRONTIER

    # -------------------------------------------------------------------------
    #  State: COMPLETE  (Bug 4: exactly-once guard)
    # -------------------------------------------------------------------------

    def _state_complete(self, now: float) -> None:
        if self._completed:
            return              # already handled - do nothing
        self._completed = True

        self._stop()
        _, _, _, pct = self._map_stats()
        self.get_logger().info(
            f"\n=== EXPLORATION COMPLETE ===\n"
            f"  Explored area  : {pct:.1f}%\n"
            f"  Total failures : {self.total_fail}"
        )

        # Cancel all timers cleanly
        self._ctrl_timer.cancel()
        self._prog_timer.cancel()
        self._svc_timer.cancel()

        self._save_map()    # P2


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
