# silvanus_bottle_picker
The robot that pays for itself
------------------------------------
---

## 🗺️ Localization & Mapping

This module manages 2D occupancy grid generation, TF frame transformations, sensor fusion, and real-time relocalization using `slam_toolbox`.

### 📅 Phase 1: Simulation Mapping & Frame Setup (Oct 1 – Oct 14)
- [ ] **URDF & TF Tree Verification**
  - Verify complete TF chain in Gazebo: `map` ➔ `odom` ➔ `base_link` ➔ `laser` / `imu`.
- [ ] **`slam_toolbox` Simulation Configuration**
  - Configure `slam_toolbox` in `async_online` mode mapping incoming 2D LiDAR scans (`/scan`).
  - Tune map resolution (0.05m), maximum laser range, and scan matcher parameters.
- [ ] **Map Persistence Testing**
  - Verify map saving and loading services (`map_saver_cli` / `slam_toolbox` serialized maps).

### 📅 Phase 2: Physical Hardware Bringup on Silvanus Base (Oct 15 – Oct 28)
- [ ] **Sensor Data Integration**
  - Verify live `/scan` and `/imu/data` streams from `scout_ros2` / `ugv_sdk` drivers.
  - Calibrate physical LiDAR placement and update wheel base transformations in URDF.
- [ ] **Campus Mapping Run**
  - Perform mapping runs around the ELTE IK testing grounds to produce baseline 2D maps.

### 📅 Phase 3: Relocalization & Outdoor Robustness (Oct 29 – Nov 30)
- [ ] **Relocalization Mode Setup**
  - Configure `slam_toolbox` for offline localization using pre-built maps.
- [ ] **Wheel-Slip & Terrain Mitigation**
  - Evaluate IMU + wheel odometry drift on grass and uneven terrain using `robot_localization` (EKF).
  - Test map updates when dynamic obstacles (e.g., pedestrians, temporary bins) are present.

---

## 🎯 Planning & Path Following

This module handles high-level target routing, Nav2 path execution, mobile manipulator docking within the xArm6 workspace envelope, and optional RL / Human-Aware local navigation layers.

### 📅 Phase 1: Base Kinematics & Team ROS 2 Interfaces (Oct 1 – Oct 14)
- [ ] **Skid-Steer Motion Control**
  - Validate non-holonomic velocity response (`cmd_vel`) using `ros2_control`.
- [ ] **Interface Definitions (Topic Contracts)**
  - Subscribed: `/trash_target_pose` (`geometry_msgs/PoseStamped`) from Perception.
  - Subscribed: `/arm_execution_status` (`std_msgs/String`) from Arm.
  - Published: `/base_in_position` (`std_msgs/Bool`) to trigger picking/dumping actions.

### 📅 Phase 2: Nav2 Core & Workspace Envelope Docking (Oct 15 – Oct 28)
- [ ] **Nav2 Stack Configuration**
  - Configure **Nav2 MPPI Controller** (Model Predictive Path Integral) tuned for skid-steer outdoor dynamics.
  - Define custom footprint parameters in `nav2_params.yaml` accounting for xArm6 payload clearance.
- [ ] **Terminal Approach & Docking Policy**
  - Build precision approach node positioning the base $0.4\text{ m} - 0.6\text{ m}$ away from detected trash for optimal xArm6 dexterity.
- [ ] **Central Bin Offloading Logic**
  - Implement onboard bin capacity tracker to route the base to the Central Bin when full.

### 📅 Phase 3: System Integration & End-to-End Pipeline (Oct 29 – Nov 4)
- [ ] **End-to-End Autonomous Cycle (Milestone 1)**
  - Validate full loop: *Detect Trash* ➔ *Navigate to Target* ➔ *Trigger Pick* ➔ *Offload to Onboard Bin* ➔ *Route to Central Bin*.
- [ ] **Recovery Behaviors**
  - Configure costmap clearing, backup maneuvers, and obstacle recovery behavior trees.

### 📅 Phase 4: RL & Human-Aware Extension Layer (Nov 5 – Nov 18)
- [ ] **Human-Aware Navigation Integration**
  - *Option A:* Integrate Social Navigation costmap layers for dynamic pedestrian distance halos.
  - *Option B:* Integrate pre-trained RL local planner (ONNX policy inference node via PyTorch / C++ API).
- [ ] **Benchmarking & Evaluation**
  - Compare Standard Nav2 MPPI vs. Human-Aware/RL Layer across path smoothness, clearance distance, and time-to-goal metrics.

### 📅 Phase 5: Outdoor Field Validation & Code Freeze (Nov 19 – Nov 30)
- [ ] **Outdoor Surface Testing**
  - Benchmark motion planning on gravel, grass, and incline paths around campus.
- [ ] **System Launch Consolidation**
  - Build unified bringup launch architecture (`ros2 launch silvanus_navigation system_bringup.launch.py`).
- [ ] **Hard Code Freeze (Nov 30, 23:59)**
  - Final repository cleanup and tagging before documentation phase (Dec 1).

---

## Perception: Bottle Recognition & Approach
**Bottle Recognition & Approach** 

This section documents the iterative development of the vision and approach module for the Silvanus robot.

### Update: [01/10/26] - Environment Setup & First Detections

* **Environment Setup:** Successfully initialized and configured the workspace for the bottle detection algorithms.
* **Initial Model Testing:** Deployed the first approximate model for bottle recognition.

![Environment for bottle detection test](images/bottle_test_1.png)
*Caption: Environment detection in gazebo.*

* **Troubleshooting:** Encountered a deployment blocker where the model was `Not working due to a GPU error` (specifically: Problem by setting up with the CUDA environment and unable to run YOLO).
* **Validation:** After resolving the GPU issue, successfully tested the model using random sample images.

#### Detection Results:
*(Below are the initial detection tests after resolving the GPU error)*

![First bottle detection test](images/bottle_test_2.jpg)
*Caption: Additional testing to verify model accuracy on random images.*

### Update: [Current Date] - Detection Positioning & 3D Spatial Integration

We have established and integrated a dedicated configuration file that commands the robotic arm to automatically navigate to a predefined "detection position." After conducting multiple preliminary tests in the simulation, we deliberately selected this specific posture because it currently provides the camera with an optimal, unobstructed field of view of the workspace. This angle significantly enhances the reliability of the bottle recognition algorithm. While this static position serves as our current baseline for approaching the target, it provides a highly stable foundation for our vision system.

In parallel with optimizing the detection posture, we are actively working on introducing 3D position estimation within the Gazebo simulation environment. By mapping the 2D bounding boxes generated by our vision model into 3D spatial coordinates (X, Y, Z), our goal is to allow the system to accurately calculate the bottle's exact physical location relative to the robot. This 3D mapping is a critical prerequisite for the upcoming grasping phase.

![Optimal Robot Position for Detection](images/bottle_detector_pos.png)
*Caption: The selected robotic posture optimized for bottle detection within the simulation.*

---

## 🤖 Grasp Pipeline (`grasp-pipeline` branch)

This module finds a bottle and grasps it with the xArm6 by its body, closing across it.
- **Detection:** YOLO finds the bottle in the color image (`bottle_detector`).
- **Position:** `bottle_localizer` turns the YOLO box and the median depth inside it into a 3D position. It is accurate to about 2 mm in the sim.
- **Shape:** `grasp_inference` crops the depth point cloud to the bottle. From those points it estimates whether the bottle is standing or lying, its direction and its length.
- **Arm motion:** `pick_bottle` plans a grasp perpendicular to the bottle's axis, moves the arm there with MoveIt, closes the gripper and lifts.

Contact-GraspNet is optional (`use_graspnet:=true`). It is then used only when no body grasp is reachable.

**Status:** the arm reaches the bottle's body and closes on it, but in Gazebo the bottle still slips out when lifted (see Known limitations).

### How it works

```
Gazebo / D435 ──/camera/color/image_raw──▶ bottle_detector (YOLO)
      │                                          │ /bottles/detections
      │                       ┌──────────────────┴───────────────────┐
      │                       ▼                                      ▼
      ├──/camera/depth/image_raw──▶ bottle_localizer      grasp_inference ◀──/camera/depth/points
      │                   (median depth → position)   (cloud crop → standing/lying,
      │                       │                         axis, length; optional
      │              /bottles/positions                 Contact-GraspNet grasps)
      │                       │                                      │ /grasp/bottle
      │                       └──────────────▶ pick_bottle ◀─────────┘
      │                                            │ /move_action, /compute_ik, ...
      │                                            ▼
      │                                  move_group (moveit_sim) ──▶ arm controller
      │                                  pick_bottle ──────────────▶ gripper controller
```

With `use_graspnet:=true`, `grasp_inference` also asks a Contact-GraspNet server for grasps. Contact-GraspNet needs its own Python/PyTorch/CUDA stack, which clashes with ROS 2 Jazzy's Python. It therefore runs as a separate process in a conda env, and `grasp_inference` talks to it over ZeroMQ on `tcp://127.0.0.1:5555`:

- **Request:** the cropped cloud as raw `float32` bytes, shape `N x 3`, in the optical frame of `/camera/depth/points`.
- **Reply:** up to 10 grasps, best score first. Each grasp is 17 `float32` values: a row-major 4x4 homogeneous transform in that same frame, followed by its score. An empty message means no grasp was found.

The network was trained for a Franka Panda gripper (80 mm opening). It therefore prefers the narrow neck, approached from about where the camera is. That is why body grasps are the default. Changing `gripper_width` in its config doesn't change this: it only caps the reported opening width.

### Repository layout

| Path | What it is |
| --- | --- |
| `src/grasp_pipeline/grasp_pipeline/grasp_inference_node.py` | `grasp_inference`: crops the cloud, estimates the bottle's shape, optionally calls the Contact-GraspNet server |
| `src/grasp_pipeline/grasp_pipeline/pick_node.py` | `pick_bottle`: plans the body grasp, moves the arm, closes the gripper and lifts |
| `src/grasp_pipeline/launch/moveit_sim.launch.py` | MoveIt `move_group` for the simulated robot |
| `src/grasp_pipeline/config/` | MoveIt semantics (SRDF), kinematics and controller config for the sim |
| `src/bottle_perception/` | `bottle_detector` (YOLO 2D boxes) and `bottle_localizer` (3D bottle positions) |
| `grasp_server/contact_graspnet_pytorch/grasp_server.py` | Inference server for the PyTorch port (**the one in use**) |
| `grasp_server/contact_graspnet_tf/grasp_server.py` | Inference server for the original NVlabs TensorFlow version (legacy) |
| `sim/grasp_demo.sh` | Starts the whole simulated pipeline in terminal tabs (see Quick start) |
| `sim/spawn_pick_bottle.sh` | Spawns one bottle inside the arm's reach (x = 0.6 m by default) |
| `sim/spawn_bottles.sh` | Spawns three test bottles (two standing, one lying) 1.0–1.5 m in front of the robot |
| `sim/observation_pose.sh` | Moves the arm to the camera "observation" pose used for far-away detection |
| `sim/models/bottle/` | Gazebo bottle model |

### Nodes

**`grasp_inference`**

| Direction | Topic | Type |
| --- | --- | --- |
| Subscribes | `/camera/depth/points` | `sensor_msgs/PointCloud2` (must be **organized**, i.e. `height > 1`), unless `depth_topic` is set |
| Subscribes | `depth_topic` + its `camera_info` | `sensor_msgs/Image` (16-bit mm or 32-bit float m) + `CameraInfo`: the cloud is built from these instead. Use depth aligned with the color image. |
| Subscribes | `/bottles/detections` | `vision_msgs/Detection2DArray` |
| Subscribes | `/camera/color/camera_info` | `sensor_msgs/CameraInfo` (used to scale boxes from color to cloud pixels) |
| Publishes | `/grasp/bottle` | `visualization_msgs/Marker` (cylinder) in `base_footprint`: pose = centre, orientation z = axis, `scale.x` = diameter, `scale.z` = length. Also viewable in RViz. |
| Publishes | `/grasp/candidates` | `geometry_msgs/PoseArray` (only with `use_graspnet:=true`): all grasps, best score first, in the cloud's frame |
| Publishes | `/grasp/pose` | `geometry_msgs/PoseStamped` (only with `use_graspnet:=true`): the best-scoring grasp |

Contact-GraspNet grasps use its own convention: z is the approach direction, x is the direction the fingers close in, and the point between the fingers is 0.1034 m along z from the origin.

**Point cloud source:** by default it uses Gazebo's organized cloud. With `depth_topic` set, it builds the organized cloud itself from that depth image and its `camera_info` (`depth_info_topic`, default: the `camera_info` next to the depth topic). The real robot publishes no point cloud, so there it uses `/camera/camera/aligned_depth_to_color/image_raw`. Clouds are processed at most `max_rate` (5) times per second.

What it does with each cloud:
- **Detection:** uses the highest-confidence box, and only if its timestamp is within 1 s of the cloud's.
- **Crop:** keeps only points within `crop_radius` (0.10 m) of the bottle centre. The 2D box alone also includes the robot's own gripper and bumper, and Contact-GraspNet grasps those instead.
- **Shape estimate:** first finds the surface the bottle rests on (floor or table top): the lowest height where the cropped points pile up. It assumes the bottle is standing (vertical axis) if its points reach more than `lying_max_height` (0.12 m) above that surface, and lying (horizontal axis) otherwise. It needs TF from the camera to `ground_frame` (`base_footprint`, z up). The diameter comes out too large with the sim's noisy depth camera, so `pick_bottle` doesn't use it. `pick_bottle` takes the surface height from it, puts the bottle on it, and adds a 30 cm square of it to MoveIt's world when it isn't the floor.
- **Contact-GraspNet** (`use_graspnet`, default `true` in the node, `false` in `grasp-demo`): sends the crop to the server, and moves each returned grasp so the fingers are centred on the points between them (`refine_grasp`).
- **Point cloud axes:** `cloud_axes` (default `gazebo`) handles Gazebo's point cloud, which uses camera-link axes even though it is labelled with the optical frame. Use `cloud_axes:=optical` with a real RealSense.
- **Server timeout:** only one request is in flight at a time. If the server doesn't reply within 30 s, the node recreates the socket and tries again on the next cloud.

**`pick_bottle`** (run with `use_sim_time:=true` in the sim)

1. Opens the gripper and moves the arm to a side "pick view" pose. From there the camera looks at the ground 0.6 m in front of the robot, the only area the arm can reach.
2. **Locates the bottle**, using only data captured after the arm settled:
   - position: the median of `bottle_samples` (5) `/bottles/positions` from `bottle_localizer`;
   - standing or lying, direction and length: from `/grasp/bottle`;
   - radius: taken as `bottle_radius` (0.035 m).
3. **Plans a body grasp, closing across the bottle:**
   - **Standing:** approaches at `body_grasp_height` (60%) of its height, from the directions in `body_yaws` around it (relative to the arm), each with the downward tilts in `body_tilts`. A flat approach isn't reachable that low; in practice it comes from the side, tilted 20–30°.
   - **Lying:** approaches from above onto the middle (`lying_tilts`).
   - The fingertips go `body_grasp_depth` (1 cm) past the axis.
   - Candidate poses are first filtered with inverse kinematics, with the bottle in MoveIt's world, so the palm can't end up inside it. MoveIt then plans to a pre-grasp 0.10 m plus the radius back along the approach, around a slightly fatter copy of the bottle.
   - With `use_graspnet:=true`, Contact-GraspNet's grasps closest to the bottle axis are tried if no body grasp is reachable.
4. **Moves straight in, then closes with a squeeze.** In the sim the fingers only press while their trajectory is still moving, so it keeps closing slowly. If the fingers close (nearly) all the way, nothing is between them: it opens, backs straight out and fails the attempt.
5. **Lifts** `lift_height` (0.15 m) straight up, or back along the approach if straight up isn't possible. It sends this directly to the arm controller without the strict goal check, because the arm sags under the load. If the squeezing fingers keep creeping shut afterwards, the bottle slipped out.

If an attempt fails, it opens the gripper, returns to the view pose and starts again, up to `max_attempts` (30) times. It publishes the gripper target it is moving to on `/grasp/tcp_target`.

**`moveit_sim.launch.py`** starts MoveIt's `move_group` with the sim's own robot model (Scout base, mount, xArm6, gripper, camera), so plans avoid the rover body. It plans for the `xarm6` group up to `link_tcp` and sends trajectories to the sim's existing `xarm6_traj_controller`. It starts no controllers of its own. Add `rviz:=true` for RViz with the MoveIt plugin.

### Prerequisites

- Ubuntu 24.04, **ROS 2 Jazzy**, Gazebo Harmonic
- [`scout_xarm_ros2`](https://github.com/IFRoS-ELTE/scout_xarm_ros2), built and sourced (simulation, robot description, `scout_xarm_teleop`, and the vendored `xarm_moveit_config`)
- An NVIDIA GPU with CUDA and conda/miniconda. One forward pass fits in 8 GB of VRAM.
- ROS packages: `ros-jazzy-moveit`, `ros-jazzy-vision-msgs`, `ros-jazzy-cv-bridge`, `ros-jazzy-tf2-ros`, `ros-jazzy-tf2-geometry-msgs`, `ros-jazzy-sensor-msgs-py`, `python3-zmq`

### 1. One-time setup

**a) Contact-GraspNet server (conda env)**

```bash
git clone https://github.com/elchun/contact_graspnet_pytorch.git ~/contact_graspnet_pytorch
cd ~/contact_graspnet_pytorch
conda env create -f contact_graspnet_env.yml -n cgn_torch
conda activate cgn_torch
pip install -e . pyzmq

# The server loads the model relative to its own location, so it must live in the repo root.
# Copy it again whenever grasp_server.py changes in this repo.
cp ~/workspaces/silvanus_bottle_picker_IFRoS_2026/grasp_server/contact_graspnet_pytorch/grasp_server.py .
```

The pretrained weights ship with the upstream repo. The server expects them at `checkpoints/contact_graspnet/checkpoints/model.pt` and exits with an error if they are missing.

**b) ROS workspace**

Always source `scout_xarm_ros2` before this workspace. Otherwise `scout_xarm_description` and `xarm_moveit_config` are not found.

```bash
conda deactivate                       # conda's python breaks ROS tools
cd ~/workspaces/silvanus_bottle_picker_IFRoS_2026
source /opt/ros/jazzy/setup.bash
source ~/workspaces/scout_xarm_ros2/install/setup.bash
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
```

`bottle_detector` needs `ultralytics`, which is not available through rosdep. Install it in a venv that can still see the ROS Python packages. Keep `numpy<2`, because Jazzy's `cv_bridge` is built against NumPy 1.x.

```bash
python3 -m venv --system-site-packages ~/silvanus_venv
source ~/silvanus_venv/bin/activate
pip install -r requirements.txt

colcon build --packages-select bottle_perception grasp_pipeline
```

`bottle_perception/setup.cfg` makes the entry points start with `#!/usr/bin/env python3`, so they run with whatever Python is active. Activate the venv in any terminal that runs `bottle_detector`, otherwise it can't import `ultralytics`. Don't build with `--symlink-install`: that ignores the `setup.cfg` setting and hard-codes `/usr/bin/python3`.

The YOLO weights (`yolo11n.pt`) are downloaded automatically on first run.

### 2. Running in simulation

Use `ROS_DOMAIN_ID=0` for the simulation in every ROS terminal. (The real robot uses domain 42.)

**Quick start:** `./sim/grasp_demo.sh` opens one tab per process in your current gnome-terminal window: `sim`, `setup`, `detector`, `grasp`, `localizer`, `moveit` and `pick`.
- The `setup` and `pick` tabs wait for the sim's controllers and its own arm initialisation before doing anything.
- `setup` spawns a bottle inside the arm's reach.
- `pick` then grasps it.

Other modes:
- `GRASPNET=1 ./sim/grasp_demo.sh` also starts the Contact-GraspNet `server` tab and uses its grasps as a fallback.
- `PICK=0 ./sim/grasp_demo.sh` gives the detection-only demo instead: the three far bottles, the observation pose and Contact-GraspNet, without the `localizer`, `moveit` and `pick` tabs.
- `./sim/grasp_demo.sh stop` shuts everything down.
- Paths can be overridden with `SIM_WS`, `CGN_DIR`, `CGN_ENV`, `VENV`, `WORLD` and `DOMAIN`.

To run the steps by hand instead, start every ROS terminal with:

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/workspaces/scout_xarm_ros2/install/setup.bash
source ~/workspaces/silvanus_bottle_picker_IFRoS_2026/install/setup.bash
export ROS_DOMAIN_ID=0
cd ~/workspaces/silvanus_bottle_picker_IFRoS_2026
```

| Terminal | Command |
| --- | --- |
| 1. Gazebo | `ros2 launch scout_xarm_gazebo sim.launch.py world:=weston_robot_empty.world` |
| 2. Bottle in reach (after the arm has finished its startup move) | `./sim/spawn_pick_bottle.sh` |
| 3. Bottle detector (also `source ~/silvanus_venv/bin/activate`) | `ros2 launch bottle_perception detector.launch.py` |
| 4. Bottle position | `ros2 run bottle_perception bottle_localizer --ros-args -p use_sim_time:=true` |
| 5. Bottle shape | `ros2 run grasp_pipeline grasp_inference --ros-args -p use_graspnet:=false` |
| 6. MoveIt | `ros2 launch grasp_pipeline moveit_sim.launch.py`, then wait for `You can start planning now!` |
| 7. Grasp | `ros2 run grasp_pipeline pick_bottle --ros-args -p use_sim_time:=true` |
| (optional) Contact-GraspNet server (conda, **no** ROS setup) | `conda activate cgn_torch && cd ~/contact_graspnet_pytorch && python grasp_server.py`; then use `use_graspnet:=true` in steps 5 and 7 |

`spawn_pick_bottle.sh` takes optional arguments: world, x, y and `lying`, for example `./sim/spawn_pick_bottle.sh weston_robot_empty 0.62 0.1 lying`.

What `pick_bottle` logs, in order:
1. `Bottle (5 positions, ...): standing at x=... y=..., length ...`
2. `Body grasp: <n> reachable pose(s)`
3. `Gripper closed on the bottle (drive_joint = ...)`
4. `Bottle grasped and lifted.` (or why the attempt failed)

Optional checks:
```bash
ros2 run rqt_image_view rqt_image_view /bottles/image_annotated   # YOLO boxes
ros2 topic echo /bottles/positions                                 # bottle position (odom frame)
ros2 topic echo /grasp/bottle                                      # bottle shape (also an RViz marker)
ros2 launch grasp_pipeline moveit_sim.launch.py rviz:=true         # MoveIt + RViz
```

### 3. Testing the Contact-GraspNet server on its own

With the server running, send it a random cloud from the `cgn_torch` env. The command prints how many grasps came back (17 floats each) and `0` if none did:

```bash
python -c "
import zmq, numpy as np
s = zmq.Context().socket(zmq.REQ)
s.setsockopt(zmq.RCVTIMEO, 60000)
s.connect('tcp://127.0.0.1:5555')
s.send(np.random.rand(2000, 3).astype(np.float32).tobytes())
print(len(s.recv()) // (17 * 4))
"
```

### Known limitations

- **Reach:** the xArm6 can only reach the ground about 0.5–0.65 m in front of `base_footprint`; the chassis is in the way closer in, and the arm is too short further out. The bottles from `spawn_bottles.sh` (1.0–1.5 m) are out of reach until navigation drives the robot closer.
- **Camera angle:** the pick view looks at the bottle from the side. From straight above, the COCO-trained YOLO sees a "sports ball", not a bottle; from behind, the front bumper hides it.
- **Reachable body grasps:** a standing bottle can only be grasped from the side, tilted about 20–30° down, and high on the body (about 60% of its height). Lower down or more tilted, the gripper's palm hits the bottle. A perfectly horizontal approach isn't reachable.
- **Sim depth noise:** the simulated depth camera has 0.1 m Gaussian depth noise (vendored `realsense.gazebo.xacro`), far more than a real D435. `bottle_localizer`'s median depth copes with this (about 2 mm error). The point-cloud shape estimate doesn't: its diameter comes out at 10–12 cm instead of 7 cm. Contact-GraspNet's grasps also vary from frame to frame.
- **Grasping in Gazebo:** the bottle still slips out of the gripper when lifted. In this sim the joints follow the controller's velocity command, so `pick_bottle` keeps the fingers closing slowly to make them press. Even so, the finger–bottle contact doesn't hold, and has once launched the bottle across the world. A reliable sim pick likely needs a grasp attach (Gazebo `DetachableJoint`) or tuned finger friction and contact stiffness in `scout_xarm_description`.
- **Contact-GraspNet randomness:** it samples its input points randomly, so the same cloud does not always give the same grasps.

### Real robot: detection and localization (not yet tested end-to-end)

The robot's bringup (`silvanus-up.sh`, ROS domain **42**) publishes the D435 under **`/camera/camera/`**:
- color: `/camera/camera/color/image_raw`, `/camera/camera/color/camera_info`;
- depth aligned to color: `/camera/camera/aligned_depth_to_color/image_raw` and its `camera_info`;
- raw depth: `/camera/camera/depth/image_rect_raw`;
- no point cloud.

The nodes' defaults are the sim's topic names, so on the robot they need the topic parameters below. (`bottle_localizer` would also cope with the raw, unaligned depth, using its `camera_info` and the depth→color TF.)

#### Robot demo (`robot-demo`)

`ROBOT=1 ./sim/grasp_demo.sh` opens the same kind of tabs as the sim demo, but for the robot: `detector`, `localizer`, `grasp` and `image` (YOLO boxes). `robot-demo` is a shell alias for it. The nodes run on your laptop and talk to the robot's camera over its Wi-Fi.

**First time only, on the laptop, while it still has internet** (the robot's Wi-Fi has none):

1. Do the [one-time setup](#1-one-time-setup) above, except a) (Contact-GraspNet isn't used on the robot). The script refuses to start unless both workspaces are built, even in robot mode:
   - `~/workspaces/scout_xarm_ros2` built (`install/setup.bash` exists), or set `SIM_WS`;
   - this workspace built with `colcon build` (no `--symlink-install`);
   - the venv at `~/silvanus_venv` with `requirements.txt` installed, or set `VENV`.
2. Install the extra laptop packages: `rqt_image_view` for the `image` tab, and `gnome-terminal`, which the script opens its tabs in.
   ```bash
   sudo apt install ros-jazzy-rqt-image-view gnome-terminal
   ```
3. Download the YOLO weights into the repo root, where the `detector` tab looks for them. Also draw one result, which makes Ultralytics download the font it draws the boxes with. Without internet neither download can happen later.
   ```bash
   conda deactivate 2>/dev/null; source ~/silvanus_venv/bin/activate
   cd ~/workspaces/silvanus_bottle_picker_IFRoS_2026
   python3 -c "from ultralytics import YOLO; import numpy as np; YOLO('yolo11n.pt').predict(np.zeros((64, 64, 3), 'uint8'), verbose=False)[0].plot()"
   ls yolo11n.pt
   ```
4. Create the aliases. Add them to `~/.bashrc`, then open a new terminal (or `source ~/.bashrc`):
   ```bash
   alias robot-demo='ROBOT=1 ~/workspaces/silvanus_bottle_picker_IFRoS_2026/sim/grasp_demo.sh'
   alias robot-demo-stop='~/workspaces/silvanus_bottle_picker_IFRoS_2026/sim/grasp_demo.sh stop'
   ```
   Check them with `type robot-demo`.

**Every time:**

1. **Start the robot's stack.** On the robot (its own screen, or `ssh agilex@192.168.1.102`), see `silvanus_robot/STUDENT_GUIDE.md`:
   ```bash
   cd /opt/silvanus
   ./silvanus-up.sh --no-base                      # camera + lidar, chassis can't move
   ./silvanus-up.sh --no-base --arm=<control-box-ip>  # also the arm: needed for correct positions, see below
   ```
   The D435 is mounted on the arm. Without `--arm`, nothing publishes the arm's real joint angles: the bringup fills them with zeros, so TF places the camera where it would be with the arm at zero. Positions in `base_footprint` are then only right if the arm really is in that pose.
2. **Connect the laptop to the robot's Wi-Fi** and check the robot's address. It comes from DHCP and can change (`hostname -I` on the robot):
   ```bash
   ping -c 3 192.168.1.102
   ```
3. **Run the demo** from a gnome-terminal window. The tabs open in that window:
   ```bash
   robot-demo                            # robot at 192.168.1.102
   ROBOT_IP=192.168.1.105 robot-demo     # robot at another address
   ```
4. **Check it works:**
   - `detector` logs `... FPS | botellas: N`. Around 0 FPS means the images aren't getting across the Wi-Fi.
   - `image` shows the YOLO boxes.
   - `localizer` logs `bottle 0: x=... y=... z=... m in "base_footprint"`.
   - `grasp` stays quiet while it works (it only warns). Its bottle estimate is on `/grasp/bottle`.
5. **Stop** with `robot-demo-stop` (it stops the laptop-side nodes only; the robot's stack keeps running).

**Grasping with the real arm (`PICK=1 robot-demo`):** this adds a `pick` tab that runs `pick_bottle` on the robot's own MoveIt and AG-95. It needs `silvanus-up.sh --arm=<control-box-ip>` with the gripper enabled (no `--no-gripper`).
1. Move the arm so the camera sees the bottle, and check `detector` and `localizer` report it.
2. Clear the area and keep the emergency stop in reach. `Ctrl-C` is not an emergency stop.
3. In the `pick` tab, press Enter. The tab waits for that before anything moves.

What it does differently from the sim:
- **No view pose** (`use_view_pose:=false`): the arm doesn't move until the bottle is located. It grasps from wherever it is as soon as `bottle_samples` positions have arrived. After a failed attempt it stays where the failure left it (e.g. backed out to the pre-grasp), so the camera must still see the bottle from there for the next attempt; otherwise the node keeps waiting for the bottle.
- **Gripper point:** the robot has no `link_tcp`. MoveIt moves `link_eef`, with the point between the fingertips at `tcp_offset` = 0.19 m along its z (the URDF's `grasp_link`), not rotated: the real AG-95 closes along `link_eef`'s y, as `link_tcp` does in the sim.
  - The robot's URDF has the AG-95 mounted 90° from the real one (`AG95_MOUNT_RPY` defaults to `0 0 0` in `silvanus-up.sh`), so in RViz and in MoveIt's collision checks its fingers close along x. The grasp orientation doesn't depend on that model, but the collision checks do. Starting the robot with the measured mount, e.g. `AG95_MOUNT_RPY="0 0 1.5708" ./silvanus-up.sh ...`, fixes the model; `tcp_offset` stays the same, as it is relative to `link_eef`.
  - The pad centres are at about 0.17 m. If the fingers close in front of the bottle or only catch it with their tips, change the 0.19 in `sim/grasp_demo.sh`.
- **Gripper:** `GripperCommand` on `/dh_ag95_gripper_controller/gripper_cmd`, joint `left_outer_knuckle_joint` (0 open, 0.93 closed). The AG-95 keeps pressing by itself, so there is no sim-style squeeze. "Closed on nothing" and the slip check (`slip_tolerance:=0.02`) work from the finger position as in the sim.
- **Speed:** `velocity_scaling:=0.1`, a third of the sim's.
- **Grasp directions:** a standing bottle is approached only horizontally from the front: straight away from the arm's base (yaw 0), then 15° and 30° either side (`body_yaws:='[0.0, 15.0, -15.0, 30.0, -30.0]'`, `body_tilts:='[0.0]'`). Never from the sides or behind, which the arm can't reach anyway. A lying bottle only straight down (`lying_tilts:='[0.0]'`). For yaw 0 only, change `body_yaws` to `'[0.0]'` in `sim/grasp_demo.sh`.
- **Drop in the box:** after the lift it carries the bottle to the pose over the box on the robot's back and opens the gripper there: `drop_joints_deg:='[-185.0, -37.0, -53.0, 1.0, 85.0, 1.0]'` (joint1..joint6 in degrees; empty, the sim's default, = no drop). While it carries the bottle, the bottle is attached to the gripper in MoveIt's model, so the path keeps it clear of the robot's body. The box itself isn't in the robot's URDF, so MoveIt doesn't avoid it: watch the first runs. If the move to the drop pose fails, it doesn't retry: it logs `Could not drop the bottle` and keeps holding it.
- **Home pose:** `home_joints_deg:='[-6.5, -37.2, -52.8, 0.9, 54.1, 1.5]'` (joint1..joint6 in degrees; empty, the sim's default, = no home pose). The arm moves there at the start, at the beginning of every retry after a failed attempt, after it gives up, and after the drop. With no view pose it looks for the bottle from there, so the camera must see the bottle from the home pose.
- **Keeps picking:** after dropping a bottle and going home it waits for the next one and picks it too, until you stop it with Ctrl-C (`continuous`, default true; only with `drop_joints_deg` set). It stops on its own if `max_attempts` attempts in a row fail on the same bottle.
- **MoveIt planning:** `planning_attempts` (default 30) plans per motion request, run in parallel within `planning_time` (5 s); the shortest valid one is used. These only affect planning to the pre-grasp and back. A grasp rejected by the IK check ("0 reachable pose(s)") never reaches the planner.
- **IK time:** `ik_timeout:=0.2` s per candidate pose (sim: 0.05). Near the edge of the reach MoveIt's IK needs longer to find a solution.
- **All candidates checked and logged** (`check_all_candidates:=true`; the sim stops after `max_candidates` reachable ones). Each attempt logs one line per candidate: its status, approach `yaw` (0 = straight away from the arm's base) and `tilt` (below horizontal), the fingertip point, the pre-grasp, and the wrist (`link_eef`) with its distance from `link_base`. Checking all 45 standing-bottle candidates can take up to about a minute.
- **Bottle radius:** still the parameter `bottle_radius` (3.5 cm). For a thinner bottle add e.g. `-p bottle_radius:=0.025` to the pick command.

**Seeing why a grasp is rejected:** every attempt publishes the bottle it planned around (blue cylinder) and each body-grasp candidate on `/grasp/body_candidates` (RViz: *Add → By topic → MarkerArray*, fixed frame `base_footprint`). Each arrow runs from the pre-grasp to the point between the fingertips: green reachable, yellow pre-grasp unreachable, orange in collision, red out of reach, grey not checked. The label above the bottle gives its horizontal distance from `link_base`. The robot's IK finds most grasps for a standing bottle about 0.45–0.55 m from `link_base` (x ≈ 0.65–0.75 in `base_footprint`), and none beyond about 0.65 m. To check the bottle estimate against the real bottle, add a *DepthCloud* display on `/camera/camera/aligned_depth_to_color/image_raw`.

Not yet tested on the robot: the first runs should be watched closely, with the arm slowed down further if needed.

What the tabs get:
- **Environment:** the same as the sim demo, plus `ROS_DOMAIN_ID=42`, `ROS_STATIC_PEERS=192.168.1.102` (`ROBOT_IP`), `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, and `ROS_LOCALHOST_ONLY` unset.
- **Not started:** `sim`, `setup` and `moveit` (the robot's own bringup runs the camera, arm and MoveIt), and `pick` unless `PICK=1`.
- Each tab stays open as a shell after its command exits: press Up + Enter to run it again.

By hand, in every terminal after the usual sourcing:

```bash
export ROS_DOMAIN_ID=42 ROS_STATIC_PEERS=192.168.1.102 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
ros2 launch bottle_perception detector.launch.py use_sim_time:=false image_topic:=/camera/camera/color/image_raw
ros2 run bottle_perception bottle_localizer --ros-args \
  -p depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  -p depth_info_topic:=/camera/camera/aligned_depth_to_color/camera_info \
  -p camera_info_topic:=/camera/camera/color/camera_info -p target_frame:=base_footprint
ros2 run rqt_image_view rqt_image_view /bottles/image_annotated
```

- **Frame:** the localizer publishes in `odom`, which only exists while the base driver runs. With `--no-base`, add `-p target_frame:=base_footprint`.
- **Python:** both nodes pick the right Python themselves, even with conda's `(base)` active. `bottle_localizer` restarts under the system Python. `bottle_detector` restarts under the venv at `~/silvanus_venv` (or `$SILVANUS_VENV`).
- **Wi-Fi:** raw camera images are big, so robot mode subscribes to them reliably (`reliable:=true`) and accepts detections up to 5 s old (`detection_max_age:=5.0` on `grasp_inference`). If it is still slow, the real fix is on the robot: a lower camera resolution, or compressed images.
- **YOLO weights:** the robot has no internet, so they must already be downloaded (they are after one run on your laptop). Run the nodes on your laptop, connected to the robot's network.
- **`grasp_inference`** (the `grasp` tab): the robot publishes no point cloud, so it builds one from the aligned depth image:
  ```bash
  ros2 run grasp_pipeline grasp_inference --ros-args -p use_graspnet:=false \
    -p depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    -r /camera/color/camera_info:=/camera/camera/color/camera_info
  ```
- **Don't run `moveit_sim.launch.py` against the robot:** it would start a second `move_group` next to the robot's own (`silvanus_robot/scout_xarm_moveit_config`).
- **`pick_bottle` on the robot** needs the robot parameters from `sim/grasp_demo.sh` (see [Grasping with the real arm](#robot-demo-robot-demo)). With its sim defaults it waits for the sim's gripper controller and never moves.

### Troubleshooting

| Symptom | Cause / fix |
| --- | --- |
| `package 'scout_xarm_description' not found` (or `xarm_moveit_config`) | `scout_xarm_ros2/install/setup.bash` wasn't sourced before this workspace. |
| `No reply from Contact-GraspNet server within 30 s` | The server isn't running or is stuck. Check `ss -tlnp \| grep 5555` and the server terminal. |
| `Waiting for /bottles/detections and /camera/color/camera_info...` | The detector isn't running, or `ROS_DOMAIN_ID` differs between terminals. |
| `No bottle detected.` | YOLO doesn't recognise the bottle from this camera angle, or it isn't in view. Watch `/bottles/image_annotated`. |
| `Latest bottle detection is ... s older than this point cloud` | The detector gets images slowly; its tab shows the FPS. On the real robot over Wi-Fi, raw 1280x720 frames (2.7 MB, hundreds of UDP fragments) are almost all lost with best-effort QoS, giving 0.0–0.1 FPS. Use `reliable:=true` on the detector, localizer and `grasp_inference` (robot mode does), and raise `detection_max_age` (robot mode: 5 s). Otherwise check whether YOLO runs on the CPU (`device` parameter). |
| `point cloud is unorganized` | The camera publishes a flat cloud (`height == 1`). The 2D box crop needs an organized cloud. |
| `Not enough valid depth points in bounding box` | The bottle is too far away, too small, or outside the depth range. Move the arm or the bottle closer. |
| `pick_bottle` keeps `Waiting for the bottle on /bottles/positions` | `bottle_localizer` isn't running, or YOLO doesn't see the bottle from the pick view pose. |
| `pick_bottle`: `Body grasp: 0 reachable pose(s)` / `MoveIt could not reach any of the grasps` | The bottle is outside the reachable area (see Known limitations). |
| `pick_bottle`: `gripper closed on nothing` / `the bottle slipped out while lifting` | The grasp missed, or the sim's contact physics let the bottle go (see Known limitations). |
| `pick_bottle` keeps waiting for `/move_action` | `moveit_sim.launch.py` isn't running, or is on a different `ROS_DOMAIN_ID`. |
| CUDA out-of-memory on the server | Close other GPU users. `forward_passes` is already 1. |
| `bottle_localizer`: `No module named 'numpy'` (older builds) | conda's `(base)` Python was used. Rebuild `bottle_perception`; the node now restarts under the system Python by itself. |
| `ModuleNotFoundError: No module named 'ultralytics'` | Either the venv isn't active, or the package was built with `--symlink-install` (check `head -1 install/bottle_perception/lib/bottle_perception/bottle_detector`). Remove `build/bottle_perception` and `install/bottle_perception`, then rebuild without that flag. |
| `ImportError` / NumPy errors from `cv_bridge` | NumPy 2 is installed in the venv. Reinstall with `pip install "numpy<2"`. |

### TODO
- [x] Hand the grasp to MoveIt for the xArm6.
- [x] Body grasp perpendicular to the bottle, located with `bottle_localizer`; Contact-GraspNet as an optional fallback.
- [ ] Make the lift hold in the sim (grasp attach or finger friction/contact tuning in `scout_xarm_description`).
- [ ] Realistic depth noise in the sim camera (through our own package; the vendored tree is read-only).
- [ ] Validate on the real D435 and robot, with the AG-95 gripper.
