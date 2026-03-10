#!/usr/bin/env bash
# ==============================================================================
#  install_sim_deps.sh
#  Ubuntu 24.04  |  ROS2 Jazzy  |  Gazebo Harmonic
#  Self-contained: installs everything needed, no leo_gz packages required.
# ==============================================================================
set -e

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# ── Sanity checks ─────────────────────────────────────────────────────────────
[[ "$(lsb_release -cs 2>/dev/null)" == "noble" ]] || warn "Expected Ubuntu 24.04 (noble)"
[[ -n "$ROS_DISTRO" ]] || error "Source ROS2 first:  source /opt/ros/jazzy/setup.bash"
[[ "$ROS_DISTRO" == "jazzy" ]] || warn "Expected ROS2 Jazzy, got $ROS_DISTRO"

info "=== Step 1: Core ROS2 packages ==="
sudo apt-get update -qq
sudo apt-get install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-msgs \
  ros-jazzy-nav2-map-server \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  python3-numpy \
  python3-colcon-common-extensions

info "=== Step 2: Gazebo Harmonic ==="
sudo apt-get install -y gz-harmonic || {
  warn "gz-harmonic not in default repos - adding OSRF apt source..."
  sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    | sudo tee /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg > /dev/null
  echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
  sudo apt-get update -qq
  sudo apt-get install -y gz-harmonic
}

info "=== Step 3: ROS-Gazebo bridge (ros_gz) ==="
sudo apt-get install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-interfaces

info "=== Step 4: RPLidar (for real robot launch) ==="
sudo apt-get install -y ros-jazzy-rplidar-ros \
  || warn "rplidar-ros not available, skip (only needed for real robot)"

info "=== Step 5: Verification ==="
echo ""
check_cmd() {
  command -v "$1" &>/dev/null \
    && echo -e "  ${GREEN}OK${NC}   $1" \
    || echo -e "  ${RED}MISS${NC} $1 (may need to open new terminal)"
}
check_pkg() {
  python3 -c "import $1; print('  \033[0;32mOK\033[0m   python3-$1', $1.__version__)" \
    2>/dev/null || echo -e "  ${RED}MISS${NC} python3-$1"
}

check_cmd gz
check_cmd ros2
check_cmd colcon
check_pkg numpy

echo ""
echo "================================================================"
echo " Installation complete!"
echo ""
echo " Build the workspace:"
echo "   cd ~/ros2_ws"
echo "   colcon build --packages-select leo_exploration"
echo "   source install/setup.bash"
echo ""
echo " Launch simulation:"
echo "   ros2 launch leo_exploration sim_exploration_launch.py"
echo ""
echo " Launch simulation (no Gazebo GUI, faster):"
echo "   ros2 launch leo_exploration sim_exploration_launch.py gz_gui:=false"
echo "================================================================"
