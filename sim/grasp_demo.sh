#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# grasp_demo.sh - start the whole simulated grasp pipeline with one command
#
#   ./sim/grasp_demo.sh          # opens one tab per process in this gnome-terminal window
#   ./sim/grasp_demo.sh stop     # stops everything it started
#
# Tabs: sim (Gazebo) | setup (bottle in reach) | detector (YOLO)
#       | localizer (bottle position) | grasp (grasp_inference: bottle shape) | moveit
#       | pick (pick_bottle: body grasp and lift)
#
#   GRASPNET=1 ./sim/grasp_demo.sh  # also run the Contact-GraspNet server and use its grasps
#                                   # as a fallback when no body grasp is reachable
#
#   PICK=0 ./sim/grasp_demo.sh   # detection only: the three far bottles + observation pose,
#                                # no moveit / pick tabs
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
PICK="${PICK:-1}"
GRASPNET="${GRASPNET:-0}"

if [ "${1:-}" = "stop" ]; then
    pkill -INT -f 'ros2 launch scout_xarm_gazebo' || true
    pkill -INT -f 'ros2 launch bottle_perception' || true
    pkill -INT -f 'grasp_pipeline/grasp_inference' || true
    pkill -INT -f 'bottle_perception/bottle_localizer' || true
    pkill -INT -f 'grasp_pipeline/pick_bottle' || true
    pkill -INT -f 'ros2 launch grasp_pipeline' || true
    pkill -INT -f 'python grasp_server.py' || true
    sleep 3
    pkill -9 -f 'gz sim' || true
    echo "Stopped."
    exit 0
fi

command -v gnome-terminal >/dev/null || { echo "gnome-terminal not found"; exit 1; }
[ -f "$SIM_WS/install/setup.bash" ] || { echo "Build $SIM_WS first"; exit 1; }
[ -f "$REPO/install/setup.bash" ] || { echo "Build $REPO first (colcon build, no --symlink-install)"; exit 1; }
if [ "$GRASPNET" = 1 ] || [ "$PICK" = 0 ]; then
    [ -f "$CGN_DIR/grasp_server.py" ] || { echo "Copy grasp_server.py into $CGN_DIR first"; exit 1; }
fi

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
# (it starts ~12 s after the arm controller), so nothing else moves the arm before it is done.
# Matched by its install path (the [x] keeps pgrep from matching the waiting shell itself),
# and given up on after 60 s so a stray process with that name can't block the demo.
INIT_HOME='lib/scout_xarm_teleop/[x]arm6_init_home'
WAIT_ARM="until ros2 control list_controllers 2>/dev/null | grep -q 'xarm_gripper_traj_controller.*active'; do echo 'waiting for controllers...'; sleep 2; done; \
for i in \$(seq 30); do pgrep -f '$INIT_HOME' >/dev/null && break; sleep 1; done; \
for i in \$(seq 60); do pgrep -f '$INIT_HOME' >/dev/null || break; echo 'waiting for init_home...'; sleep 1; done"

if [ "$PICK" = 1 ]; then
    # Only the ground ~0.5-0.65 m in front of the robot is in reach; pick_bottle moves the
    # arm to its own view pose, so no observation pose here
    SETUP_CMD="$WAIT_ARM; ./sim/spawn_pick_bottle.sh $WORLD"
else
    SETUP_CMD="$WAIT_ARM; ./sim/spawn_bottles.sh $WORLD && ./sim/observation_pose.sh"
fi

# Contact-GraspNet: always in the detection-only demo, otherwise only with GRASPNET=1
if [ "$PICK" = 0 ] || [ "$GRASPNET" = 1 ]; then USE_CGN=true; else USE_CGN=false; fi

tab sim      "$ROS_ENV" "ros2 launch scout_xarm_gazebo sim.launch.py world:=$WORLD.world"
tab setup    "$ROS_ENV" "$SETUP_CMD"
if [ "$USE_CGN" = true ]; then
    tab server "conda activate $CGN_ENV
cd '$CGN_DIR'" "python grasp_server.py"
fi
tab detector "$ROS_ENV
source '$VENV/bin/activate'" "ros2 launch bottle_perception detector.launch.py"
tab grasp    "$ROS_ENV" "ros2 run grasp_pipeline grasp_inference --ros-args -p use_graspnet:=$USE_CGN"
if [ "$PICK" = 1 ]; then
    tab localizer "$ROS_ENV" "ros2 run bottle_perception bottle_localizer --ros-args -p use_sim_time:=true"
    tab moveit "$ROS_ENV" "ros2 launch grasp_pipeline moveit_sim.launch.py"
    tab pick   "$ROS_ENV" "$WAIT_ARM; ros2 run grasp_pipeline pick_bottle --ros-args -p use_sim_time:=true -p use_graspnet:=$USE_CGN"
fi
