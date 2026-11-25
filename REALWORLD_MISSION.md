# Silvanus real-world bottle mission (backup)

Snapshot of the laptop-side grasp + nav pipeline used with the physical
Silvanus Scout + xArm + AG95 + D435 (ROS 2 Jazzy, `ROS_DOMAIN_ID=42`).

Best bag so far (kept **local**, not in git):  
`~/bags/mission_20261125-130425`

## What this repo contains

| Path | Role |
|------|------|
| `src/bottle_perception/` | YOLOE detector + bottle localizer |
| `src/grasp_pipeline/` | `grasp_inference`, `nav_to_bottle`, `pick_bottle`, `arm_pose` |
| `sim/grasp_demo.sh` | Perception (+ optional pick) tabs for `ROBOT=1` |
| `sim/mission_robot.sh` | One-command laptop mission: check + perception + nav |
| `src/grasp_pipeline/config/arm_poses_robot.yaml` | Survey / near joint poses (deg) |
| `src/grasp_pipeline/config/pick_robot.yaml` | Real-robot pick parameters |

Robot bringup / Nav2 launch live on the robot (`silvanus-up`, `navigation_hardware.launch.py`) — see `silvanus_robot-master/docs/RUN_EXPLORE_THEN_NAV.md` in the robot docs tree if present on the machine.

## Quick laptop run

```bash
# after robot: silvanus-up --arm  +  nav slam:=false map:=lab  +  2D Pose Estimate
cd ~/jazzy_ws/src/silvanus_bottle_picker_IFRoS_2026-grasping_realworld_yoloe_merged
source install/setup.bash
SIM_WS=$HOME/jazzy_ws VENV=$HOME/venvs/yolo ./sim/mission_robot.sh check
SIM_WS=$HOME/jazzy_ws VENV=$HOME/venvs/yolo ./sim/mission_robot.sh
# Enter on pick tab only after: Goal succeeded — base at standoff, arm at near pose
```

## `nav_to_bottle` guards (post bag debug)

- Max bottle range (default 2.5 m)
- Stability count before committing
- Reject goals outside `/map`
- Lock first goal (no retarget spam on abort)
- Detector `confidence:=0.45` in robot `grasp_demo`

## Do not commit

- `build/`, `install/`, `log/`
- `*.pt` weights, bags, camera dumps
- Anything under `~/bags/`
