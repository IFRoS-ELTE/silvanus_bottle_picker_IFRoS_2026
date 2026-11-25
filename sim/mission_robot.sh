#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# mission_robot.sh — one-command laptop side for the real Silvanus mission
#
#   ./sim/mission_robot.sh           # perception + nav_to_bottle (+ pick tab waiting)
#   ./sim/mission_robot.sh stop      # stop laptop nodes started by this / grasp_demo
#   ./sim/mission_robot.sh nav       # only nav_to_bottle (perception already running)
#   ./sim/mission_robot.sh check     # print readiness checklist
#
# Robot side (still separate — needs SSH / docker on the Nitro or robot PC):
#   1) silvanus-up.sh --arm=...
#   2) navigation_hardware.launch.py slam:=false map:=/bags/maps/lab.yaml
#   3) RViz → 2D Pose Estimate once (scan matches walls)
#
# Then on the laptop run this script.
# ---------------------------------------------------------------------------
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_WS="${SIM_WS:-$HOME/jazzy_ws}"
VENV="${VENV:-$HOME/venvs/yolo}"
DOMAIN="${DOMAIN:-42}"
ROBOT_IP="${ROBOT_IP:-192.168.1.102}"
MAP_HINT="${MAP_HINT:-/bags/maps/lab.yaml}"
STANDOFF="${STANDOFF:-0.55}"
PICK="${PICK:-1}"
PARAMS="$REPO/install/grasp_pipeline/share/grasp_pipeline/config/arm_poses_robot.yaml"

ros_env() {
    # shellcheck disable=SC1090
    source /opt/ros/jazzy/setup.bash
    # shellcheck disable=SC1090
    source "$SIM_WS/install/setup.bash" 2>/dev/null || true
    # shellcheck disable=SC1090
    source "$REPO/install/setup.bash"
    export ROS_DOMAIN_ID="$DOMAIN"
    export ROS_STATIC_PEERS="$ROBOT_IP"
    unset ROS_LOCALHOST_ONLY
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    export SILVANUS_VENV="$VENV"
    export PATH="$VENV/bin:$PATH"
}

cmd_stop() {
    "$REPO/sim/grasp_demo.sh" stop || true
    pkill -INT -f 'grasp_pipeline/nav_to_bottle' 2>/dev/null || true
    pkill -INT -f 'grasp_pipeline nav_to_bottle' 2>/dev/null || true
    echo "Laptop mission stopped."
}

cmd_check() {
    ros_env
    echo "=== Laptop readiness (domain=$DOMAIN peer=$ROBOT_IP) ==="
    echo -n "map→base_footprint: "
    if timeout 3 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null | head -n 5 | grep -q Translation; then
        echo "OK"
    else
        echo "MISSING — set 2D Pose Estimate / wait for AMCL"
    fi
    echo -n "bt_navigator: "
    ros2 lifecycle get /bt_navigator 2>/dev/null || echo "not found"
    echo -n "/move_action: "
    if ros2 action list 2>/dev/null | grep -q '/move_action'; then echo "OK"; else echo "MISSING (silvanus-up --arm?)"; fi
    echo -n "/bottles/positions: "
    if timeout 3 ros2 topic hz /bottles/positions 2>&1 | head -n 2 | grep -q average; then
        echo "OK"
    else
        echo "no data (start perception / detector)"
    fi
    echo -n "/map: "
    if timeout 3 ros2 topic echo /map --once --field info >/dev/null 2>&1; then
        echo "OK"
    else
        echo "MISSING (nav launch with slam:=false map:=...)"
    fi
    echo
    echo "Robot side still required:"
    echo "  ssh → ./silvanus-up.sh --arm=<xarm_ip>"
    echo "  docker → ros2 launch .../navigation_hardware.launch.py slam:=false map:=$MAP_HINT"
    echo "  RViz → 2D Pose Estimate once"
}

cmd_nav() {
    ros_env
    [ -f "$PARAMS" ] || { echo "Missing $PARAMS — colcon build grasp_pipeline"; exit 1; }
    echo "Starting nav_to_bottle (standoff=$STANDOFF). Ctrl-C to stop."
    exec ros2 run grasp_pipeline nav_to_bottle --ros-args \
        --params-file "$PARAMS" \
        -p standoff_distance:="$STANDOFF" \
        -p max_bottle_distance:=2.5 \
        -p stable_count:=3 \
        -p max_nav_retries:=2
}

cmd_all() {
    cmd_check
    echo
    echo "Starting perception tabs (ROBOT=1 PICK=$PICK)..."
    SIM_WS="$SIM_WS" VENV="$VENV" ROBOT=1 PICK="$PICK" DOMAIN="$DOMAIN" \
        ROBOT_IP="$ROBOT_IP" "$REPO/sim/grasp_demo.sh"
    # give detector a moment, then open nav in a new tab if gnome-terminal exists
    if command -v gnome-terminal >/dev/null; then
        rc="${XDG_RUNTIME_DIR:-/tmp}/silvanus_mission_nav.rc"
        cat > "$rc" <<EOF
source ~/.bashrc
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source '$SIM_WS/install/setup.bash' 2>/dev/null || true
source '$REPO/install/setup.bash'
export ROS_DOMAIN_ID=$DOMAIN
export ROS_STATIC_PEERS=$ROBOT_IP
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export SILVANUS_VENV='$VENV'
export PATH='$VENV/bin':\"\$PATH\"
cd '$REPO'
echo 'Wait until map TF + bottles look good, then nav starts in 8s...'
sleep 8
history -s "ros2 run grasp_pipeline nav_to_bottle ..."
ros2 run grasp_pipeline nav_to_bottle --ros-args \
  --params-file '$PARAMS' \
  -p standoff_distance:=$STANDOFF \
  -p max_bottle_distance:=2.5 \
  -p stable_count:=3 \
  -p max_nav_retries:=2
EOF
        gnome-terminal --tab --title="nav_to_bottle" -- bash --rcfile "$rc" -i
        echo "Opened nav_to_bottle tab (starts after 8s delay)."
    else
        echo "No gnome-terminal — run: $0 nav"
    fi
    echo
    echo "When nav logs 'Goal succeeded ... near pose', press Enter in the pick tab."
}

case "${1:-all}" in
    stop)  cmd_stop ;;
    check) cmd_check ;;
    nav)   cmd_nav ;;
    all|"") cmd_all ;;
    *)
        echo "Usage: $0 [all|nav|check|stop]"
        exit 1
        ;;
esac
