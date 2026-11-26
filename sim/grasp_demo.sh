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
#   ROBOT=1 ./sim/grasp_demo.sh  # the REAL robot instead of Gazebo: every tab exports
#                                # ROS_DOMAIN_ID=42, ROS_STATIC_PEERS=$ROBOT_IP,
#                                # ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET, unsets
#                                # ROS_LOCALHOST_ONLY; runs detector, localizer and grasp
#                                # (no sim/setup/moveit tabs: the robot runs its own MoveIt)
#   ROBOT=1 PICK=1 ./sim/grasp_demo.sh  # also a pick tab: pick_bottle on the real arm and
#                                       # AG-95, started only after you press Enter in it
#   ROBOT=1 PICK=server ./sim/grasp_demo.sh  # pick tab in action-server mode for the behaviour
#                                       # tree (silvanus_bt): after ONE Enter it serves
#                                       # /pick_bottle and only moves when the tree asks
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
# Prefer the lab YOLO venv when present; else the README default name.
if [ -z "${VENV:-}" ]; then
    if [ -x "$HOME/venvs/yolo/bin/python3" ]; then
        VENV="$HOME/venvs/yolo"
    else
        VENV="$HOME/silvanus_venv"
    fi
fi
WORLD="${WORLD:-weston_robot_empty}"
ROBOT="${ROBOT:-0}"
ROBOT_IP="${ROBOT_IP:-192.168.1.102}"
if [ "$ROBOT" = 1 ]; then DOMAIN="${DOMAIN:-42}"; else DOMAIN="${DOMAIN:-0}"; fi
# The real arm only moves when asked for: PICK defaults to 0 in robot mode
if [ "$ROBOT" = 1 ]; then PICK="${PICK:-0}"; else PICK="${PICK:-1}"; fi
GRASPNET="${GRASPNET:-0}"
# Each tab runs `bash --rcfile $RC_DIR/<name>.rc`, which is also how stop finds the tabs
RC_DIR="${XDG_RUNTIME_DIR:-/tmp}/silvanus_grasp_demo"

if [ "${1:-}" = "stop" ]; then
    pkill -INT -f 'ros2 launch scout_xarm_gazebo' || true
    pkill -INT -f 'ros2 launch bottle_perception' || true
    pkill -INT -f 'grasp_pipeline/grasp_inference' || true
    pkill -INT -f 'bottle_perception/bottle_localizer' || true
    pkill -INT -f 'rqt_image_view' || true
    pkill -INT -f 'grasp_pipeline/pick_bottle' || true
    pkill -INT -f 'grasp_pipeline/nav_to_bottle' || true
    pkill -INT -f 'ros2 launch grasp_pipeline' || true
    pkill -INT -f 'python grasp_server.py' || true
    sleep 3
    pkill -9 -f 'gz sim' || true
    echo "Stopped."
    # Close the tabs: an interactive bash ignores SIGTERM, so hang it up (the tab closes when
    # its shell exits). If this is run from one of them, that tab closes too.
    pkill -HUP -f "bash --rcfile $RC_DIR/" || true
    exit 0
fi

command -v gnome-terminal >/dev/null || { echo "gnome-terminal not found"; exit 1; }
[ -f "$SIM_WS/install/setup.bash" ] || { echo "Build $SIM_WS first"; exit 1; }
[ -f "$REPO/install/setup.bash" ] || { echo "Build $REPO first (colcon build, no --symlink-install)"; exit 1; }
if [ "$ROBOT" != 1 ] && { [ "$GRASPNET" = 1 ] || [ "$PICK" = 0 ]; }; then
    [ -f "$CGN_DIR/grasp_server.py" ] || { echo "Copy grasp_server.py into $CGN_DIR first"; exit 1; }
fi

mkdir -p "$RC_DIR"

# ROS environment shared by all ROS tabs. ~/.bashrc auto-activates conda base, whose
# python3 would shadow the system/venv one, so it is deactivated first.
# bottle_detector re-execs under $SILVANUS_VENV (defaults to ~/silvanus_venv). Point it at
# the same tree as VENV so the detector tab does not die when that default path is missing.
ROS_ENV="conda deactivate 2>/dev/null || true
conda deactivate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source '$SIM_WS/install/setup.bash'
source '$REPO/install/setup.bash'
export ROS_DOMAIN_ID=$DOMAIN
export SILVANUS_VENV='$VENV'
export PATH='$VENV/bin':\"\$PATH\"
cd '$REPO'"
if [ "$ROBOT" = 1 ]; then
    # Reach the robot's ROS graph over the network
    ROS_ENV="$ROS_ENV
export ROS_STATIC_PEERS=$ROBOT_IP
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET"
fi

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

if [ "$ROBOT" = 1 ]; then
    # Real robot: its own bringup runs the camera, arm and MoveIt (silvanus-up.sh --arm), so
    # no moveit tab: moveit_sim.launch.py would start a second move_group next to the robot's.
    # The robot's D435 driver publishes under /camera/camera/ (namespace + camera name)
    CAM=/camera/camera
    # Images cross Wi-Fi: subscribe reliably (best effort loses nearly every big frame), and
    # accept detections up to 5 s older than the depth (the camera and bottle are still)
    # conf 0.45 cuts the ~0.25–0.40 ghost boxes that drove out-of-map nav goals in the bag
    tab detector "$ROS_ENV
source '$VENV/bin/activate'
export SILVANUS_VENV='$VENV'" "ros2 launch bottle_perception detector.launch.py use_sim_time:=false image_topic:=$CAM/color/image_raw reliable:=true confidence:=0.45"
    tab localizer "$ROS_ENV" "ros2 run bottle_perception bottle_localizer --ros-args -p depth_topic:=$CAM/aligned_depth_to_color/image_raw -p depth_info_topic:=$CAM/aligned_depth_to_color/camera_info -p camera_info_topic:=$CAM/color/camera_info -p target_frame:=base_footprint -p reliable:=true"
    # The robot publishes no point cloud: grasp_inference builds it from the aligned depth
    tab grasp "$ROS_ENV" "ros2 run grasp_pipeline grasp_inference --ros-args -p use_graspnet:=false -p depth_topic:=$CAM/aligned_depth_to_color/image_raw -p reliable:=true -p detection_max_age:=5.0 -r /camera/color/camera_info:=$CAM/color/camera_info"
    tab image "$ROS_ENV" "ros2 run rqt_image_view rqt_image_view /bottles/image_annotated"
    if [ "$PICK" = 1 ] || [ "$PICK" = server ]; then
        # pick_bottle on the robot's MoveIt and AG-95, with the robot parameters (home and drop
        # poses, gripper, grasp angles) from src/grasp_pipeline/config/pick_robot.yaml, the same
        # file the robot's Docker image uses. Edit it there, then colcon build.
        # Waits for Enter: the arm moves.
        PICK_CMD="ros2 run grasp_pipeline pick_bottle --ros-args --params-file '$REPO/install/grasp_pipeline/share/grasp_pipeline/config/pick_robot.yaml'"
        if [ "$PICK" = server ]; then
            # Action-server mode: the behaviour tree sends one /pick_bottle goal per bottle
            tab pick "$ROS_ENV" "read -rp 'pick_bottle server MOVES THE REAL ARM whenever the behaviour tree asks. E-stop in reach? Enter to start, Ctrl-C to cancel. ' && $PICK_CMD -p server:=true"
        else
            tab pick "$ROS_ENV" "read -rp 'pick_bottle MOVES THE REAL ARM. E-stop in reach, arm looking at the bottle? Enter to start, Ctrl-C to cancel. ' && $PICK_CMD"
        fi
    fi
    echo "Robot mode (ROS_DOMAIN_ID=$DOMAIN, robot $ROBOT_IP): detector, localizer, grasp, image$([ "$PICK" != 0 ] && [ -n "$PICK" ] && echo ", pick ($PICK)")."
    [ "$PICK" = 1 ] || [ "$PICK" = server ] || echo "No pick tab: run with PICK=1 to grasp with the real arm."
    exit 0
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
