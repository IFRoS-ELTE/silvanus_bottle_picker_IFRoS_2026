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

This module turns a bottle detection into a 6-DoF grasp pose for the xArm6. YOLO finds the bottle in the color image. The `grasp_pipeline` node crops the depth point cloud to that bounding box. A Contact-GraspNet server then predicts the best grasp on the cropped points.

### How it works

```
Gazebo / D435 ──/camera/color/image_raw──▶ bottle_detector (YOLO)
      │                                          │
      │                                 /bottles/detections
      │                                          ▼
      ├──/camera/depth/points───────▶ grasp_inference (ROS 2 node)
      └──/camera/color/camera_info──▶   crop cloud to best bbox
                                                 │  ZeroMQ, tcp://127.0.0.1:5555
                                                 ▼
                                   grasp_server.py (conda env, GPU)
                                   Contact-GraspNet → best 4x4 grasp
```

Contact-GraspNet needs its own Python/PyTorch/CUDA stack, which clashes with ROS 2 Jazzy's Python. It therefore runs as a separate process in a conda env, and the ROS node talks to it over ZeroMQ:

- **Request:** the cropped cloud as raw `float32` bytes, shape `N x 3`, in the frame of `/camera/depth/points`.
- **Reply:** the highest-scoring grasp as a 4x4 `float32` homogeneous transform in that same camera frame, or an empty message if no grasp was found.

### Repository layout

| Path | What it is |
| --- | --- |
| `src/grasp_pipeline/` | ROS 2 client node `grasp_inference` (crops the cloud, calls the server) |
| `src/bottle_perception/` | `bottle_detector` (YOLO 2D boxes) and `bottle_localizer` (3D bottle positions) |
| `grasp_server/contact_graspnet_pytorch/grasp_server.py` | Inference server for the PyTorch port (**the one in use**) |
| `grasp_server/contact_graspnet_tf/grasp_server.py` | Inference server for the original NVlabs TensorFlow version (legacy) |
| `sim/spawn_bottles.sh` | Spawns three test bottles (two standing, one lying) in front of the robot |
| `sim/observation_pose.sh` | Moves the arm to the camera "observation" pose used for detection |
| `sim/models/bottle/` | Gazebo bottle model |

### ROS interface — `grasp_inference`

| Direction | Topic | Type |
| --- | --- | --- |
| Subscribes | `/camera/depth/points` | `sensor_msgs/PointCloud2` (must be **organized**, i.e. `height > 1`) |
| Subscribes | `/bottles/detections` | `vision_msgs/Detection2DArray` |
| Subscribes | `/camera/color/camera_info` | `sensor_msgs/CameraInfo` (used to scale boxes from color to cloud pixels) |

Behaviour:
- Uses the highest-confidence detection. A detection is used only if its timestamp is within 1 s of the point cloud's.
- Skips the request if fewer than 100 valid points fall inside the box.
- Only one request is in flight at a time. If the server doesn't reply within 30 s, the node recreates the socket and tries again on the next cloud.
- The resulting grasp pose is currently **only logged**; publishing it to TF / MoveIt is still TODO.

### Prerequisites

- Ubuntu 24.04, **ROS 2 Jazzy**, Gazebo Harmonic
- [`scout_xarm_ros2`](https://github.com/IFRoS-ELTE/scout_xarm_ros2), built and sourced (simulation, robot description, `scout_xarm_teleop`)
- An NVIDIA GPU with CUDA and conda/miniconda. One forward pass fits in 8 GB of VRAM.
- ROS packages: `ros-jazzy-vision-msgs`, `ros-jazzy-cv-bridge`, `ros-jazzy-tf2-ros`, `ros-jazzy-tf2-geometry-msgs`, `ros-jazzy-sensor-msgs-py`, `python3-zmq`

### 1. One-time setup

**a) Contact-GraspNet server (conda env)**

```bash
git clone https://github.com/elchun/contact_graspnet_pytorch.git ~/contact_graspnet_pytorch
cd ~/contact_graspnet_pytorch
conda env create -f contact_graspnet_env.yml -n cgn_torch
conda activate cgn_torch
pip install -e . pyzmq

# The server loads the model relative to its own location, so it must live in the repo root
cp ~/workspaces/silvanus_bottle_picker_IFRoS_2026/grasp_server/contact_graspnet_pytorch/grasp_server.py .
```

The pretrained weights ship with the upstream repo. The server expects them at `checkpoints/contact_graspnet/checkpoints/model.pt` and exits with an error if they are missing.

**b) ROS workspace**

```bash
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

colcon build --symlink-install --packages-select bottle_perception grasp_pipeline
```

The node entry points start with `#!/usr/bin/env python3`, so activate the venv in any terminal that runs `bottle_detector`. Otherwise it can't import `ultralytics`.

The YOLO weights (`yolo11n.pt`) are downloaded automatically on first run.

### 2. Running in simulation

Use `ROS_DOMAIN_ID=0` for the simulation in every ROS terminal. (The real robot uses domain 42.)

**Terminal 1: Gazebo**
```bash
source ~/workspaces/scout_xarm_ros2/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch scout_xarm_gazebo sim.launch.py world:=weston_robot_empty.world
```

**Terminal 2: bottles + arm observation pose**
```bash
source ~/workspaces/scout_xarm_ros2/install/setup.bash
export ROS_DOMAIN_ID=0
cd ~/workspaces/silvanus_bottle_picker_IFRoS_2026
./sim/spawn_bottles.sh            # pass a world name (without .world) if you launched a different one
./sim/observation_pose.sh
```

**Terminal 3: grasp server (conda, not ROS)**
```bash
conda activate cgn_torch
cd ~/contact_graspnet_pytorch
python grasp_server.py
# wait for: "Server ready on 5555"
```

**Terminal 4: bottle detector**
```bash
source ~/silvanus_venv/bin/activate
cd ~/workspaces/silvanus_bottle_picker_IFRoS_2026
source install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch bottle_perception detector.launch.py
```

**Terminal 5: grasp client**
```bash
cd ~/workspaces/silvanus_bottle_picker_IFRoS_2026
source install/setup.bash
export ROS_DOMAIN_ID=0
ros2 run grasp_pipeline grasp_inference
```

When it works, the client logs `Received Grasp Pose:` followed by a 4x4 matrix. The server logs `Success. <n> grasps, best score: ...`.

Optional checks:
```bash
ros2 run rqt_image_view rqt_image_view /bottles/image_annotated   # YOLO boxes
ros2 run bottle_perception bottle_localizer                        # 3D bottle positions + RViz markers
```

### 3. Testing the server on its own

With the server running, send it a random cloud from the `cgn_torch` env. The command prints `64` if a grasp came back (16 floats × 4 bytes) and `0` if none did:

```bash
python -c "
import zmq, numpy as np
s = zmq.Context().socket(zmq.REQ)
s.setsockopt(zmq.RCVTIMEO, 60000)
s.connect('tcp://127.0.0.1:5555')
s.send(np.random.rand(2000, 3).astype(np.float32).tobytes())
print(len(s.recv()))
"
```

### Real D435 camera (not yet tested end-to-end)

Start the driver with aligned depth and an organized point cloud, then remap the client's topics to the driver's names:

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true pointcloud.ordered_pc:=true
ros2 run grasp_pipeline grasp_inference --ros-args \
  -r /camera/depth/points:=/camera/camera/depth/color/points \
  -r /camera/color/camera_info:=/camera/camera/color/camera_info
```

Run the detector with `use_sim_time:=false` and the matching `image_topic`.

### Troubleshooting

| Symptom | Cause / fix |
| --- | --- |
| `No reply from Contact-GraspNet server within 30 s` | The server isn't running or is stuck. Check `ss -tlnp \| grep 5555` and the server terminal. |
| `Waiting for /bottles/detections and /camera/color/camera_info...` | The detector isn't running, or `ROS_DOMAIN_ID` differs between terminals. |
| `Latest bottle detection is too old for this point cloud` | The detector is slow or stopped. Check whether YOLO is running on the CPU (`device` parameter). |
| `point cloud is unorganized` | The camera publishes a flat cloud (`height == 1`). The 2D box crop needs an organized cloud. |
| `Not enough valid depth points in bounding box` | The bottle is too far away, too small, or outside the depth range. Move the arm or the bottle closer. |
| CUDA out-of-memory on the server | Close other GPU users. `forward_passes` is already 1. |
| `ImportError` / NumPy errors from `cv_bridge` | NumPy 2 is installed in the venv. Reinstall with `pip install "numpy<2"`. |

### TODO
- [ ] Publish the grasp pose (TF + `PoseStamped`) in the robot's frame.
- [ ] Hand the grasp to MoveIt for the xArm6 pick.
- [ ] Filter grasps against the gripper's reachable workspace and approach direction.
- [ ] Validate on the real D435 and robot.
