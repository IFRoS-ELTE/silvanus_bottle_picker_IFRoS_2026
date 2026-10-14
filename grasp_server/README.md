# Contact-GraspNet server

Grasp inference runs outside ROS, in its own conda environment, and talks to the
`grasp_pipeline` ROS 2 node (`src/grasp_pipeline`) over ZeroMQ on `tcp://127.0.0.1:5555`.

The node sends a cropped point cloud as raw `float32` bytes (N x 3) and gets back the
best grasp as a 4x4 `float32` matrix, or an empty reply when no grasp is found.

Each `grasp_server.py` has to be copied into the root of the matching upstream repo,
because it loads the model code and checkpoints relative to its own location:

| Folder | Upstream repo | Conda env |
| --- | --- | --- |
| `contact_graspnet_pytorch/` | https://github.com/elchun/contact_graspnet_pytorch | `cgn_torch` |
| `contact_graspnet_tf/` | https://github.com/NVlabs/contact_graspnet | `contact_graspnet_env` |

The PyTorch server is the one currently in use.

## Running

Server:

```bash
conda activate cgn_torch
cd ~/contact_graspnet_pytorch
python grasp_server.py
```

Client:

```bash
export ROS_DOMAIN_ID=0
colcon build --packages-select grasp_pipeline
source install/setup.bash
ros2 run grasp_pipeline grasp_inference
```
## To be updated!! to create te conda environment


Below is the updated list of changes and requirements made during setup to ensure compatibility and avoid common environment conflicts.

## 1. Environment Setup for Contact GraspNet
Instead of using the original `contact_graspnet_env.yml` (which uses Python 3.9, torch 2.0.1, and includes `mayavi` which often fails to compile), set up a clean Conda environment with the following commands:

```bash
conda create -n cgn_torch python=3.10 -y
conda activate cgn_torch
pip install torch==2.11.0 --index-url https://download.pytorch.org/whl/cu128
pip install numpy==2.2.6 scipy==1.15.3 trimesh==5.1.1 pyrender==0.1.45 \
    opencv-python-headless==5.0.0.93 pyzmq==27.2.0 pyyaml==6.0.3 tqdm imageio pillow \
    pyglet==2.1.16 pyopengl==3.1.0 freetype-py
pip install -e . --no-deps --no-build-isolation
```

> **Tip:** It is recommended to save these dependencies inside `grasp_server/requirements_cgn.txt` for easier installation.

## 2. Resolving Python Environment Conflicts
To prevent path and library conflicts between system packages and Conda:
* Run `conda config --set auto_activate_base false`
* Do **not** source or load ROS automatically in the server terminal profile.

## 3. Building the ROS/Colcon Workspace
When compiling the workspace with the virtual environment active, make sure to use `python3 -m colcon build` instead of standard `colcon build`:

```bash
python3 -m colcon build --symlink-install
```

## 4. YOLO Environment Configuration
Use a single dedicated virtual environment for YOLO (the naming convention is flexible, e.g., `~/silvanus_venv` or `~/venvs/yolo`).