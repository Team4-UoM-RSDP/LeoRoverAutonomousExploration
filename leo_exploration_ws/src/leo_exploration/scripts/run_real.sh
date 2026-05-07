#!/usr/bin/env bash
# Quiet one-command bring-up for the real Leo Rover split deployment.
#
# This script intentionally leaves exploration_launch.py in charge of the ROS
# node ordering. It only wraps it with quiet logging and external readiness
# checks so a demo operator sees stage progress instead of raw ROS chatter.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
PI_IP="${PI_IP:-192.168.8.2}"
LOCAL_IP="${LOCAL_IP:-auto}"
NETWORK_INTERFACE="${NETWORK_INTERFACE:-auto}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-42}"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
ODOM_TOPIC="${ODOM_TOPIC:-/merged_odom}"
RVIZ="${RVIZ:-true}"
EXPLORER="${EXPLORER:-true}"
LAUNCH_LIDAR="${LAUNCH_LIDAR:-true}"
CMD_VEL_OUT_TOPIC="${CMD_VEL_OUT_TOPIC:-/cmd_vel_relay}"
BUILD_MODE="${BUILD_MODE:-auto}"     # auto | always | never
STARTUP_TIMEOUT="${STARTUP_TIMEOUT:-180}"
FOLLOW_LOG="${FOLLOW_LOG:-true}"
SERIAL_CHMOD="${SERIAL_CHMOD:-true}"
SHOW_LAST_LINES="${SHOW_LAST_LINES:-80}"

EXTRA_LAUNCH_ARGS=()
START_TS="$(date +%s)"
LAUNCH_PID=""
TAIL_PID=""
LAUNCH_LOG=""
BUILD_LOG=""

usage() {
  cat <<'EOF'
Usage:
  ./run_real.sh [options]

Common options:
  --pi-ip IP                 Leo Rover Pi IP (default: 192.168.8.2)
  --serial-port PATH         RPLidar serial device (default: /dev/ttyUSB0)
  --domain-id ID             ROS_DOMAIN_ID shared with the Pi (default: 42)
  --local-ip IP|auto         Local IP for CycloneDDS peers (default: auto)
  --network-interface IF|auto Local network interface (default: auto)
  --rviz true|false          Launch RViz (default: true)
  --no-rviz                  Shortcut for --rviz false
  --explorer true|false      Start autonomous explorer after Nav2 (default: true)
  --cmd-vel-out TOPIC        Final velocity topic (default: /cmd_vel_relay)
  --cmd-vel-debug            Dry-run output to /cmd_vel_debug
  --scan-topic TOPIC         LaserScan topic (default: /scan)
  --timeout SECONDS          Per-stage startup timeout (default: 180)
  --build                    Force colcon build before launch
  --no-build                 Never build; require install/setup.bash
  --no-follow-log            Keep running quietly after readiness
  --                         Pass remaining args directly to ros2 launch

Environment overrides use the same uppercase names, for example:
  PI_IP=192.168.8.2 SERIAL_PORT=/dev/ttyUSB0 ./run_real.sh
EOF
}

elapsed() {
  local now diff mins secs
  now="$(date +%s)"
  diff=$((now - START_TS))
  mins=$((diff / 60))
  secs=$((diff % 60))
  printf "%02d:%02d" "$mins" "$secs"
}

say() {
  printf '[%s] %s\n' "$(elapsed)" "$*"
}

ok() {
  say "[OK] $*"
}

warn() {
  say "[WARN] $*"
}

show_tail() {
  local file="$1"
  [[ -f "$file" ]] || return 0
  say "Last $SHOW_LAST_LINES lines from $file:"
  tail -n "$SHOW_LAST_LINES" "$file" || true
}

stop_launch() {
  if [[ -n "${TAIL_PID:-}" ]]; then
    kill "$TAIL_PID" 2>/dev/null || true
    TAIL_PID=""
  fi

  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    say "Stopping real-robot launch..."
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}

fail() {
  say "[FAIL] $*"
  if [[ -n "${BUILD_LOG:-}" ]]; then
    show_tail "$BUILD_LOG"
  fi
  if [[ -n "${LAUNCH_LOG:-}" ]]; then
    show_tail "$LAUNCH_LOG"
  fi
  stop_launch
  exit 1
}

on_signal() {
  say "Interrupted by operator."
  stop_launch
  exit 130
}

trap on_signal INT TERM

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pi-ip)
      PI_IP="$2"; shift 2 ;;
    --serial-port)
      SERIAL_PORT="$2"; shift 2 ;;
    --domain-id|--ros-domain-id)
      ROS_DOMAIN_ID_VALUE="$2"; shift 2 ;;
    --local-ip)
      LOCAL_IP="$2"; shift 2 ;;
    --network-interface)
      NETWORK_INTERFACE="$2"; shift 2 ;;
    --rviz)
      RVIZ="$2"; shift 2 ;;
    --no-rviz)
      RVIZ="false"; shift ;;
    --explorer)
      EXPLORER="$2"; shift 2 ;;
    --no-explorer)
      EXPLORER="false"; shift ;;
    --launch-lidar)
      LAUNCH_LIDAR="$2"; shift 2 ;;
    --no-lidar)
      LAUNCH_LIDAR="false"; shift ;;
    --cmd-vel-out)
      CMD_VEL_OUT_TOPIC="$2"; shift 2 ;;
    --cmd-vel-debug)
      CMD_VEL_OUT_TOPIC="/cmd_vel_debug"; shift ;;
    --scan-topic)
      SCAN_TOPIC="$2"; shift 2 ;;
    --odom-topic)
      ODOM_TOPIC="$2"; shift 2 ;;
    --timeout)
      STARTUP_TIMEOUT="$2"; shift 2 ;;
    --build)
      BUILD_MODE="always"; shift ;;
    --no-build)
      BUILD_MODE="never"; shift ;;
    --follow-log)
      FOLLOW_LOG="true"; shift ;;
    --no-follow-log)
      FOLLOW_LOG="false"; shift ;;
    --no-serial-chmod)
      SERIAL_CHMOD="false"; shift ;;
    -h|--help)
      usage; exit 0 ;;
    --)
      shift
      EXTRA_LAUNCH_ARGS+=("$@")
      break ;;
    *)
      fail "Unknown option: $1" ;;
  esac
done

command_exists() {
  command -v "$1" >/dev/null 2>&1
}

require_command() {
  command_exists "$1" || fail "Missing command: $1"
}

source_ros() {
  if [[ -z "${ROS_DISTRO:-}" ]]; then
    [[ -f "$ROS_SETUP" ]] || fail "ROS setup not found: $ROS_SETUP"
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
  fi

  [[ "${ROS_DISTRO:-}" == "jazzy" ]] || warn "Expected ROS 2 Jazzy, got ROS_DISTRO=${ROS_DISTRO:-unset}"
  ok "ROS environment ready (${ROS_DISTRO:-unknown})"
}

maybe_build_workspace() {
  local setup_file="$WS_ROOT/install/setup.bash"
  local should_build="false"

  case "$BUILD_MODE" in
    always) should_build="true" ;;
    auto)
      [[ -f "$setup_file" ]] || should_build="true"
      ;;
    never) should_build="false" ;;
    *) fail "Invalid BUILD_MODE=$BUILD_MODE" ;;
  esac

  mkdir -p "$HOME/.ros/leo_demo_logs"
  BUILD_LOG="$HOME/.ros/leo_demo_logs/build_real_$(date +%Y%m%d_%H%M%S).log"

  if [[ "$should_build" == "true" ]]; then
    require_command colcon
    say "Build: colcon build --packages-select leo_exploration (log: $BUILD_LOG)"
    (
      cd "$WS_ROOT"
      colcon build --symlink-install --packages-select leo_exploration
    ) >"$BUILD_LOG" 2>&1 || fail "Build failed"
    ok "Build complete"
  else
    [[ -f "$setup_file" ]] || fail "install/setup.bash is missing; rerun with --build"
    ok "Build skipped (use --build to force rebuild)"
  fi

  # shellcheck disable=SC1090
  source "$setup_file"
  ok "Workspace overlay sourced"
}

check_ros_packages() {
  local missing=()
  local packages=(
    leo_exploration
    rplidar_ros
    slam_toolbox
    tf2_ros
    nav2_controller
    nav2_planner
    nav2_behaviors
    nav2_bt_navigator
    nav2_collision_monitor
    nav2_velocity_smoother
    nav2_lifecycle_manager
    rmw_cyclonedds_cpp
  )

  for pkg in "${packages[@]}"; do
    ros2 pkg prefix "$pkg" >/dev/null 2>&1 || missing+=("$pkg")
  done

  if (( ${#missing[@]} > 0 )); then
    fail "Missing ROS packages: ${missing[*]}. Install deps before the demo."
  fi

  python3 - <<'PY' >/dev/null 2>&1 || fail "Missing python3-psutil; install python3-psutil"
import psutil
PY

  if [[ "$RVIZ" == "true" ]]; then
    require_command rviz2
  fi

  ok "Required ROS/Python packages are visible"
}

resolve_network() {
  local route_output parsed_ip parsed_if

  if [[ "$LOCAL_IP" == "auto" || "$NETWORK_INTERFACE" == "auto" ]]; then
    require_command ip
    route_output="$(ip -4 route get "$PI_IP" 2>/dev/null || true)"
    [[ -n "$route_output" ]] || fail "Cannot route to Pi IP $PI_IP. Check WiFi/network before launch."

    parsed_ip="$(sed -n 's/.* src \([^ ]*\).*/\1/p' <<<"$route_output" | head -n 1)"
    parsed_if="$(sed -n 's/.* dev \([^ ]*\).*/\1/p' <<<"$route_output" | head -n 1)"

    [[ "$LOCAL_IP" != "auto" ]] || LOCAL_IP="$parsed_ip"
    [[ "$NETWORK_INTERFACE" != "auto" ]] || NETWORK_INTERFACE="$parsed_if"
  fi

  [[ -n "$LOCAL_IP" && "$LOCAL_IP" != "auto" ]] || fail "Could not detect local IP; pass --local-ip"
  [[ -n "$NETWORK_INTERFACE" && "$NETWORK_INTERFACE" != "auto" ]] || fail "Could not detect network interface; pass --network-interface"

  ok "DDS network: local $LOCAL_IP on $NETWORK_INTERFACE, Pi $PI_IP, domain $ROS_DOMAIN_ID_VALUE"
}

write_cyclonedds_config() {
  local dds_config
  dds_config="/tmp/leo_cyclonedds_${ROS_DOMAIN_ID_VALUE}_${LOCAL_IP//./_}_${PI_IP//./_}_run_real.xml"

  cat >"$dds_config" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="$NETWORK_INTERFACE" priority="default" multicast="default" />
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
      <Peers>
        <Peer Address="$PI_IP" />
        <Peer Address="$LOCAL_IP" />
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF

  export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"
  export ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE"
  export ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET"
  export ROS_IP="$LOCAL_IP"
  export CYCLONEDDS_URI="file://$dds_config"
  unset FASTRTPS_DEFAULT_PROFILES_FILE || true
  unset ROS_DISCOVERY_SERVER || true

  ok "CycloneDDS config written"
}

check_serial_port() {
  if [[ "$LAUNCH_LIDAR" != "true" ]]; then
    ok "Local lidar launch disabled"
    return
  fi

  [[ -e "$SERIAL_PORT" ]] || fail "Serial port not found: $SERIAL_PORT"

  if [[ ! -r "$SERIAL_PORT" || ! -w "$SERIAL_PORT" ]]; then
    if [[ "$SERIAL_CHMOD" == "true" ]]; then
      say "Serial permission: fixing $SERIAL_PORT with sudo chmod a+rw"
      sudo chmod a+rw "$SERIAL_PORT" || fail "Could not change serial permissions for $SERIAL_PORT"
    else
      fail "Serial port is not readable/writable: $SERIAL_PORT"
    fi
  fi

  ok "Serial port ready: $SERIAL_PORT"
}

launch_stack() {
  mkdir -p "$HOME/.ros/leo_demo_logs"
  LAUNCH_LOG="$HOME/.ros/leo_demo_logs/real_launch_$(date +%Y%m%d_%H%M%S).log"

  local launch_args=(
    "pi_ip:=$PI_IP"
    "local_ip:=$LOCAL_IP"
    "network_interface:=$NETWORK_INTERFACE"
    "ros_domain_id:=$ROS_DOMAIN_ID_VALUE"
    "serial_port:=$SERIAL_PORT"
    "rviz:=$RVIZ"
    "explorer:=$EXPLORER"
    "launch_lidar:=$LAUNCH_LIDAR"
    "scan_topic:=$SCAN_TOPIC"
    "cmd_vel_out_topic:=$CMD_VEL_OUT_TOPIC"
  )

  say "Launch: starting real robot stack (log: $LAUNCH_LOG)"
  (
    cd "$WS_ROOT"
    ros2 launch leo_exploration exploration_launch.py "${launch_args[@]}" "${EXTRA_LAUNCH_ARGS[@]}"
  ) >"$LAUNCH_LOG" 2>&1 &
  LAUNCH_PID="$!"

  sleep 2
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    fail "ros2 launch exited immediately"
  fi
  ok "Launch process running (pid $LAUNCH_PID)"
}

launch_alive() {
  [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null
}

topic_visible() {
  timeout 4s ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

topic_once() {
  local topic="$1"
  timeout 5s ros2 topic echo --once "$topic" >/dev/null 2>&1
}

map_once() {
  local topic="$1"
  timeout 6s ros2 topic echo --once --qos-durability transient_local "$topic" >/dev/null 2>&1 \
    || topic_visible "$topic"
}

tf_ready() {
  local target="$1"
  local source="$2"
  local output
  output="$(timeout 4s ros2 run tf2_ros tf2_echo "$target" "$source" 2>&1 || true)"
  grep -Eq "Translation:|At time" <<<"$output"
}

all_nav2_active() {
  local node state
  local nodes=(
    controller_server
    planner_server
    behavior_server
    velocity_smoother
    collision_monitor
    bt_navigator
  )

  for node in "${nodes[@]}"; do
    state="$(timeout 3s ros2 lifecycle get "/$node" 2>/dev/null || true)"
    grep -qi "active" <<<"$state" || return 1
  done
}

action_ready() {
  timeout 4s ros2 action list 2>/dev/null | grep -Fxq "/navigate_to_pose"
}

node_visible() {
  timeout 4s ros2 node list 2>/dev/null | grep -Fxq "$1"
}

wait_for() {
  local label="$1"
  local timeout_seconds="$2"
  shift 2

  say "Waiting: $label"
  local start now
  start="$(date +%s)"
  while true; do
    if "$@"; then
      ok "$label"
      return 0
    fi

    launch_alive || fail "Launch stopped while waiting for: $label"

    now="$(date +%s)"
    if (( now - start >= timeout_seconds )); then
      fail "Timeout while waiting for: $label"
    fi
    sleep 1
  done
}

wait_for_startup() {
  wait_for "Pi base odometry on $ODOM_TOPIC" "$STARTUP_TIMEOUT" topic_once "$ODOM_TOPIC"
  wait_for "Local lidar scans on $SCAN_TOPIC" "$STARTUP_TIMEOUT" topic_once "$SCAN_TOPIC"
  wait_for "TF odom -> laser" "$STARTUP_TIMEOUT" tf_ready "odom" "laser"
  wait_for "SLAM map on /map" "$STARTUP_TIMEOUT" map_once "/map"
  wait_for "TF map -> base_link" "$STARTUP_TIMEOUT" tf_ready "map" "base_link"
  wait_for "Nav2 lifecycle nodes active" "$STARTUP_TIMEOUT" all_nav2_active
  wait_for "Nav2 /navigate_to_pose action server" "$STARTUP_TIMEOUT" action_ready

  if [[ "$EXPLORER" == "true" ]]; then
    wait_for "frontier_explorer node running" "$STARTUP_TIMEOUT" node_visible "/frontier_explorer"
  fi
}

follow_or_wait() {
  if [[ "$FOLLOW_LOG" == "true" ]]; then
    ok "Real robot stack is ready. Live navigation logs start now. Ctrl-C stops the stack."
    tail -n 0 -F "$LAUNCH_LOG" &
    TAIL_PID="$!"
  else
    ok "Real robot stack is ready. Keeping launch attached quietly. Ctrl-C stops the stack."
  fi

  set +e
  wait "$LAUNCH_PID"
  local status="$?"
  set -e
  if [[ -n "${TAIL_PID:-}" ]]; then
    kill "$TAIL_PID" 2>/dev/null || true
  fi

  say "Launch exited with status $status"
  exit "$status"
}

main() {
  [[ "$(uname -s)" == "Linux" ]] || fail "This real-robot launcher must run on the Ubuntu NUC, not Windows."
  require_command timeout
  require_command grep
  require_command sed
  require_command python3

  say "Leo Rover real-robot quiet startup"
  ok "Workspace: $WS_ROOT"

  source_ros
  maybe_build_workspace
  check_ros_packages
  resolve_network
  write_cyclonedds_config
  check_serial_port
  launch_stack
  wait_for_startup
  follow_or_wait
}

main
