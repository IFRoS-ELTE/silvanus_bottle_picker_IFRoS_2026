# Contact-GraspNet server

Grasp inference runs outside ROS, in its own conda environment, and talks to the
`grasp_pipeline` ROS 2 node (`src/grasp_pipeline`) over ZeroMQ on `tcp://127.0.0.1:5555`.

The node sends a cropped point cloud as raw `float32` bytes (N x 3) and gets back the
top 10 grasps, best first, each as 17 `float32` values (a row-major 4x4 matrix followed by its score), or an empty reply when no grasp is found.

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
