#!/usr/bin/env python3
"""
Leo Rover Frontier-Based Autonomous Exploration Node
ROS2 Jazzy  |  v2.0

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

        # Bug 4: ensure _state_complete executes exactly once
        self._completed: bool = False

        # Bug 3: service readiness flags (updated by 1 Hz probe timer)
        self._svc_local_ok:  bool = False
        self._svc_global_ok: bool = False

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
        self.declare_parameter("obstacle_dist",        0.45)
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

    # -------------------------------------------------------------------------
    #  Callbacks
    # -------------------------------------------------------------------------

    def _map_cb(self, msg: OccupancyGrid)     -> None: self.map_data     = msg
    def _costmap_cb(self, msg: OccupancyGrid) -> None: self.costmap_data = msg   # P1
    def _scan_cb(self, msg: LaserScan)        -> None: self.latest_scan  = msg

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
        """
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
        if self.map_data is None:
            return []

        w   = self.map_data.info.width
        h   = self.map_data.info.height
        res = self.map_data.info.resolution
        ox  = self.map_data.info.origin.position.x
        oy  = self.map_data.info.origin.position.y

        raw     = np.array(self.map_data.data, dtype=np.int8).reshape((h, w))
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
        for sy, sx in zip(ys.tolist(), xs.tolist()):
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

        # Convert to world coordinates + P1 inflation filter
        frontiers: List[Frontier] = []
        for cid, cells in clusters.items():
            if len(cells) < self.p_min_frontier:
                continue
            mean_col = sum(c[0] for c in cells) / len(cells)
            mean_row = sum(c[1] for c in cells) / len(cells)
            wx = mean_col * res + ox
            wy = mean_row * res + oy
            if self._frontier_in_inflation(wx, wy):    # P1 filter
                continue
            frontiers.append(Frontier(wx, wy, len(cells)))

        return frontiers

    def _frontier_in_inflation(self, wx: float, wy: float) -> bool:
        """
        P1: Return True if the world point (wx, wy) falls inside an inflation
        or lethal zone according to the Nav2 global costmap.
        Frontiers inside such zones cannot be reached by the robot footprint
        without colliding, so we discard them before scoring.
        """
        cm = self.costmap_data
        if cm is None:
            return False    # costmap not yet available: allow frontier through

        res = cm.info.resolution
        ox  = cm.info.origin.position.x
        oy  = cm.info.origin.position.y
        w   = cm.info.width
        h   = cm.info.height

        col = int((wx - ox) / res)
        row = int((wy - oy) / res)

        if not (0 <= col < w and 0 <= row < h):
            return False    # outside costmap bounds: allow through

        cost = cm.data[row * w + col]
        # nav2 costmap: 0=free, 1-252=inflated, 253=inscribed, 254=lethal, 255=unknown
        # We treat cost >= COSTMAP_LETHAL_THRESH as unreachable
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

        Frontiers closer than 0.5m are FILTERED OUT entirely — sending
        a goal that close causes the controller to instantly report
        'Reached the goal' without actually moving.
        """
        # Keep visited list bounded to avoid stale penalties
        if len(self._visited) > 50:
            trimmed = list(self._visited)[-30:]
            self._visited.clear()
            self._visited.extend(trimmed)

        result: List[Frontier] = []
        for f in frontiers:
            dist = math.hypot(f.cx - rx, f.cy - ry)

            # Skip frontiers too close to robot — they cause instant
            # "reached goal" without any movement
            if dist < 0.5:
                continue

            # Information gain: normalised cluster size
            info_score = min(1.0, f.size / 60.0)

            # Distance: prefer 1.0-4.0 m range
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
        goal.pose.header.frame_id    = self.p_map_frame
        goal.pose.header.stamp       = self.get_clock().now().to_msg()
        goal.pose.pose.position.x    = fx
        goal.pose.pose.position.y    = fy
        goal.pose.pose.orientation.w = 1.0

        self._nav_done = False
        self._nav_ok   = False
        self._nav_t0   = self._now_sec()   # Bug 5: ROS clock

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
            else:
                self.consec_fail += 1
                self.total_fail  += 1
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
