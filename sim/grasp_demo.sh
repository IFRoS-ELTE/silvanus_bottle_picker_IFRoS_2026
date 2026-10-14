#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# grasp_demo.sh - start the whole simulated grasp pipeline with one command
#
#   ./sim/grasp_demo.sh          # opens one tab per process in this gnome-terminal window
#   ./sim/grasp_demo.sh stop     # stops everything it started
#
# Tabs: sim (Gazebo) | setup (bottles + observation pose) | server (Contact-GraspNet)
#       | detector (YOLO) | grasp (grasp_inference client)
#
# Each tab stays open as a normal shell with the right environment after its command
# exits; press Up + Enter to run it again.
#
# Paths can be overridden with environment variables (see the defaults below).
# ---------------------------------------------------------------------------
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_WS="${SIM_WS:-$HOME/workspaces/scout_xarm_ros2}"
CGN_DIR="${CGN_DIR:-$HOME/contact_graspnet_pytorch}"
CGN_ENV="${CGN_ENV:-cgn_torch}"
VENV="${VENV:-$HOME/silvanus_venv}"
WORLD="${WORLD:-weston_robot_empty}"
DOMAIN="${DOMAIN:-0}"

if [ "${1:-}" = "stop" ]; then
    pkill -INT -f 'ros2 launch scout_xarm_gazebo' || true
    pkill -INT -f 'ros2 launch bottle_perception' || true
    pkill -INT -f 'grasp_pipeline/grasp_inference' || true
    pkill -INT -f 'python grasp_server.py' || true
    sleep 3
    pkill -9 -f 'gz sim' || true
    echo "Stopped."
    exit 0
fi

command -v gnome-terminal >/dev/null || { echo "gnome-terminal not found"; exit 1; }
[ -f "$SIM_WS/install/setup.bash" ] || { echo "Build $SIM_WS first"; exit 1; }
[ -f "$REPO/install/setup.bash" ] || { echo "Build $REPO first (colcon build, no --symlink-install)"; exit 1; }
[ -f "$CGN_DIR/grasp_server.py" ] || { echo "Copy grasp_server.py into $CGN_DIR first"; exit 1; }

RC_DIR="${XDG_RUNTIME_DIR:-/tmp}/silvanus_grasp_demo"
mkdir -p "$RC_DIR"

# ROS environment shared by all ROS tabs. ~/.bashrc auto-activates conda base, whose
# python3 would shadow the system/venv one, so it is deactivated first.
ROS_ENV="conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source '$SIM_WS/install/setup.bash'
source '$REPO/install/setup.bash'
export ROS_DOMAIN_ID=$DOMAIN
cd '$REPO'"

# tab <name> <environment setup> <command>
tab() {
    local rc="$RC_DIR/$1.rc"
    cat > "$rc" <<EOF
source ~/.bashrc
$2
history -s $(printf '%q' "$3")
echo -e "\e[1;32m\$ $3\e[0m"
$3
EOF
    gnome-terminal --tab --title="$1" -- bash --rcfile "$rc" -i
}

# Wait for the last controller, then for the launch's own xarm6_init_home move to finish
# (it starts ~12 s after the arm controller), so it doesn't override the observation pose.
SETUP_CMD="until ros2 control list_controllers 2>/dev/null | grep -q 'xarm_gripper_traj_controller.*active'; do echo 'waiting for controllers...'; sleep 2; done; \
for i in \$(seq 30); do pgrep -f xarm6_init_home >/dev/null && break; sleep 1; done; \
while pgrep -f xarm6_init_home >/dev/null; do echo 'waiting for init_home...'; sleep 1; done; \
./sim/spawn_bottles.sh $WORLD && ./sim/observation_pose.sh"

tab sim      "$ROS_ENV" "ros2 launch scout_xarm_gazebo sim.launch.py world:=$WORLD.world"
tab setup    "$ROS_ENV" "$SETUP_CMD"
tab server   "conda activate $CGN_ENV
cd '$CGN_DIR'" "python grasp_server.py"
tab detector "$ROS_ENV
source '$VENV/bin/activate'" "ros2 launch bottle_perception detector.launch.py"
tab grasp    "$ROS_ENV" "ros2 run grasp_pipeline grasp_inference"
