#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# robot_demo.sh - bottle detection + localization against the REAL robot
#
#   ./robot/robot_demo.sh          # opens one tab per process in this gnome-terminal window
#   ./robot/robot_demo.sh stop     # stops everything it started
#
# Tabs: detector (YOLO on the D435) | localizer (bottle positions) | image (YOLO boxes)
#       | shell (a terminal with the same environment, for ros2 topic echo etc.)
#
# Every tab is set up with:
#   source $VENV/bin/activate
#   source /opt/ros/jazzy/setup.bash
#   source $WS/install/setup.bash
#   export ROS_DOMAIN_ID=42 ROS_STATIC_PEERS=$ROBOT_IP ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
#   unset ROS_LOCALHOST_ONLY
#
# Only perception runs here. moveit_sim.launch.py and pick_bottle are for the simulated arm
# and must not be run against the robot.
#
# Overridable: VENV, WS, ROBOT_IP, DOMAIN, TARGET_FRAME (frame of /bottles/positions; odom
# only exists while the robot's base driver runs).
# ---------------------------------------------------------------------------
set -euo pipefail

VENV="${VENV:-$HOME/venvs/yolo}"
# The workspace this script is in, unless overridden (e.g. WS=~/silvanus_bottle_picker)
WS="${WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
ROBOT_IP="${ROBOT_IP:-192.168.1.102}"
DOMAIN="${DOMAIN:-42}"
TARGET_FRAME="${TARGET_FRAME:-base_footprint}"

if [ "${1:-}" = "stop" ]; then
    pkill -INT -f 'ros2 launch bottle_perception' || true
    pkill -INT -f 'bottle_perception/bottle_localizer' || true
    pkill -INT -f 'rqt_image_view' || true
    echo "Stopped."
    exit 0
fi

command -v gnome-terminal >/dev/null || { echo "gnome-terminal not found"; exit 1; }
[ -f "$VENV/bin/activate" ] || { echo "No venv at $VENV (set VENV=...)"; exit 1; }
[ -f "$WS/install/setup.bash" ] || { echo "Build $WS first (colcon build, no --symlink-install)"; exit 1; }

RC_DIR="${XDG_RUNTIME_DIR:-/tmp}/silvanus_robot_demo"
mkdir -p "$RC_DIR"

# ~/.bashrc auto-activates conda base and sets its own ROS_DOMAIN_ID, so deactivate conda
# first and set the robot's network settings last
ROBOT_ENV="conda deactivate 2>/dev/null || true
source '$VENV/bin/activate'
source /opt/ros/jazzy/setup.bash
source '$WS/install/setup.bash'

export ROS_DOMAIN_ID=$DOMAIN
export ROS_STATIC_PEERS=$ROBOT_IP
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
cd '$WS'"

# tab <name> <command>; the tab stays open as a shell with the environment afterwards
tab() {
    local rc="$RC_DIR/$1.rc"
    { echo 'source ~/.bashrc'
      echo "$ROBOT_ENV"
      if [ -n "$2" ]; then
          echo "history -s $(printf '%q' "$2")"
          printf 'echo -e "\\e[1;32m$ %s\\e[0m"\n' "$2"
          echo "$2"
      fi
    } > "$rc"
    gnome-terminal --tab --title="$1" -- bash --rcfile "$rc" -i
}

tab detector  "ros2 launch bottle_perception detector.launch.py use_sim_time:=false"
tab localizer "ros2 run bottle_perception bottle_localizer --ros-args -p depth_topic:=/camera/depth/image_rect_raw -p target_frame:=$TARGET_FRAME"
tab image     "ros2 run rqt_image_view rqt_image_view /bottles/image_annotated"
tab shell     ""
