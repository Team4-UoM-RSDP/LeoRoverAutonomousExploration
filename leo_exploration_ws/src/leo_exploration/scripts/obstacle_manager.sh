#!/usr/bin/env bash
# ==============================================================================
#  obstacle_manager.sh
#  Add, move, and remove obstacles in the running Gazebo simulation.
#  Works with Gazebo Harmonic (gz-sim 8) via gz topic / gz service CLI.
#
#  Usage:
#    ./obstacle_manager.sh add   [x] [y]           # drop 0.5x0.5x0.8 box
#    ./obstacle_manager.sh wall  [x] [y] [len] [angle]  # add wall segment
#    ./obstacle_manager.sh clear                    # remove all added obstacles
#    ./obstacle_manager.sh list                     # list world models
# ==============================================================================

set -e
CMD="${1:-help}"
COUNTER_FILE="/tmp/leo_obs_counter"

# ── Helpers ───────────────────────────────────────────────────────────────────
next_id() {
  local n=1
  [ -f "$COUNTER_FILE" ] && n=$(( $(cat "$COUNTER_FILE") + 1 ))
  echo "$n" > "$COUNTER_FILE"
  echo "$n"
}

check_gz() {
  command -v gz &>/dev/null || {
    echo "ERROR: 'gz' CLI not found. Install with: sudo apt install gz-harmonic"
    exit 1
  }
}

# ── add: spawn a coloured box at (x, y) ──────────────────────────────────────
add_box() {
  local x="${1:-1.0}" y="${2:-1.0}" z=0.4
  local id; id=$(next_id)
  local name="obstacle_${id}"
  local r g b
  # Cycle colours for visual distinction
  case $(( id % 4 )) in
    0) r=0.9; g=0.3; b=0.2 ;;  # red
    1) r=0.2; g=0.5; b=0.9 ;;  # blue
    2) r=0.2; g=0.8; b=0.3 ;;  # green
    3) r=0.9; g=0.7; b=0.1 ;;  # yellow
  esac

  local sdf="<?xml version='1.0'?>
<sdf version='1.9'>
  <model name='${name}'>
    <static>true</static>
    <pose>${x} ${y} ${z} 0 0 0</pose>
    <link name='link'>
      <collision name='c'><geometry><box><size>0.5 0.5 0.8</size></box></geometry></collision>
      <visual name='v'>
        <geometry><box><size>0.5 0.5 0.8</size></box></geometry>
        <material>
          <ambient>${r} ${g} ${b} 1</ambient>
          <diffuse>${r} ${g} ${b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"

  gz service -s /world/leo_exploration_world/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "sdf: '${sdf}' name: '${name}'"

  echo "Spawned box '${name}' at (${x}, ${y})"
}

# ── wall: spawn a thin wall segment ──────────────────────────────────────────
add_wall() {
  local x="${1:-0.0}" y="${2:-0.0}" len="${3:-2.0}" angle="${4:-0.0}"
  local id; id=$(next_id)
  local name="wall_obs_${id}"

  local sdf="<?xml version='1.0'?>
<sdf version='1.9'>
  <model name='${name}'>
    <static>true</static>
    <pose>${x} ${y} 1.0 0 0 ${angle}</pose>
    <link name='link'>
      <collision name='c'><geometry><box><size>${len} 0.2 2.0</size></box></geometry></collision>
      <visual name='v'>
        <geometry><box><size>${len} 0.2 2.0</size></box></geometry>
        <material><ambient>0.55 0.5 0.45 1</ambient><diffuse>0.55 0.5 0.45 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"

  gz service -s /world/leo_exploration_world/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "sdf: '${sdf}' name: '${name}'"

  echo "Spawned wall '${name}' at (${x}, ${y}) length=${len} angle=${angle}"
}

# ── clear: remove all runtime-added obstacles ─────────────────────────────────
clear_obstacles() {
  local max=1
  [ -f "$COUNTER_FILE" ] && max=$(cat "$COUNTER_FILE")
  echo "Removing up to ${max} obstacles..."
  for i in $(seq 1 "$max"); do
    gz service -s /world/leo_exploration_world/remove \
      --reqtype gz.msgs.Entity \
      --reptype gz.msgs.Boolean \
      --timeout 2000 \
      --req "name: 'obstacle_${i}' type: 2" 2>/dev/null && echo "  removed obstacle_${i}" || true
    gz service -s /world/leo_exploration_world/remove \
      --reqtype gz.msgs.Entity \
      --reptype gz.msgs.Boolean \
      --timeout 2000 \
      --req "name: 'wall_obs_${i}' type: 2" 2>/dev/null && echo "  removed wall_obs_${i}" || true
  done
  rm -f "$COUNTER_FILE"
  echo "Done."
}

# ── list: show all models currently in simulation ────────────────────────────
list_models() {
  gz topic -e -t /world/leo_exploration_world/state -n 1 2>/dev/null \
    | grep "name:" | sort \
    || echo "Could not read world state (is Gazebo running?)"
}

# ── help ──────────────────────────────────────────────────────────────────────
show_help() {
  echo ""
  echo "  obstacle_manager.sh — runtime obstacle control for Leo Gazebo sim"
  echo ""
  echo "  Commands:"
  echo "    add  [x] [y]                — add 0.5x0.5x0.8 box at (x, y)"
  echo "    wall [x] [y] [len] [angle]  — add wall; angle in radians"
  echo "    clear                       — remove all added obstacles"
  echo "    list                        — list all models in simulation"
  echo ""
  echo "  Examples:"
  echo "    ./obstacle_manager.sh add 3.0 2.0"
  echo "    ./obstacle_manager.sh wall 0.0 3.0 3.0 1.5708   # vertical wall"
  echo "    ./obstacle_manager.sh clear"
  echo ""
  echo "  Alternative (GUI): In Gazebo window → Insert tab → box/cylinder"
  echo "    Then drag and drop wherever you want."
  echo ""
}

# ── dispatch ──────────────────────────────────────────────────────────────────
case "$CMD" in
  add)   check_gz; add_box   "${2}" "${3}" ;;
  wall)  check_gz; add_wall  "${2}" "${3}" "${4}" "${5}" ;;
  clear) check_gz; clear_obstacles ;;
  list)  check_gz; list_models ;;
  *)     show_help ;;
esac
