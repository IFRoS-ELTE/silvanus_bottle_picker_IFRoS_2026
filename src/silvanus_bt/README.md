# silvanus_bt — Silvanus behaviour tree (py_trees_ros)

Explores the environment (SLAM + explore_lite) and, when it sees a stable bottle, collects it:
pauses exploration, drives up in two steps facing the bottle, puts the arm in `near`, calls
`/pick_bottle` (grasp, lift, drop in the box) and goes back to exploring.

## Install (once, on the laptop)

```bash
sudo apt install ros-jazzy-py-trees ros-jazzy-py-trees-ros ros-jazzy-py-trees-ros-viewer
cd ~/workspaces/silvanus_bottle_picker          # always from the repo, never from ~
colcon build --packages-select silvanus_interfaces bottle_perception grasp_pipeline silvanus_bt
source install/setup.bash
```

## Run on the real robot (in this order)

1. **Robot T1**: `AG95_MOUNT_RPY="0 0 1.5708" ./silvanus-up.sh --arm=192.168.1.219` (no `--no-base`).
2. **Laptop A**: `ROBOT=1 PICK=server ./sim/grasp_demo.sh` → press Enter ONCE in the pick tab;
   it logs `Ready: serving /pick_bottle`.
3. **Laptop B**: `ros2 launch silvanus_bt bt_robot.launch.py` → waits for Nav2.
4. **Robot T2** (inside `sudo docker exec -it silvanus bash`):
   `ros2 launch /opt/bringup/navigation_hardware.launch.py explore:=true`
   → the tree logs `Tree ready`; ~10 s later the robot explores on its own. **E-stop in hand.**
5. Check before letting it go: `ros2 run tf2_ros tf2_echo map base_footprint` prints a pose.
6. Watch it: `py-trees-tree-viewer` or `py-trees-tree-watcher -b`.

### Test step by step

| What | Command |
|---|---|
| Navigation only, no arm, no exploration | `ros2 launch silvanus_bt bt_robot.launch.py use_arm:=false use_explore:=false pick_mode:=skip` |
| Everything but the pick | `... pick_mode:=skip` |
| Manual pick (old way, Enter + `/bt/pick_result`) | `... pick_mode:=wait` with `PICK=1` in grasp_demo |
| Nearest bottle instead of most confident | `... selection:=nearest` |

## The tree

```
=> Silvanus                        Parallel
   Bottles2BB                      sensors -> blackboard (always SUCCESS)
   ? Tasks                         Selector WITHOUT memory: priorities, every tick
      -> CollectBottle             PRIORITY 1
         BottleToCollect?          stable bottle? picks the most confident one
         ? Handle
            -> Attempt
               PauseExplore        False -> /explore/resume
               ? Approach          step 1: stop at 1.2 m and measure again
               ? Standoff          step 2: 0.55 m, facing the bottle
               -> ArmNear          MoveIt -> near pose
               PickBottle          action /pick_bottle (pick_bottle server:=true)
               MarkCollected       blacklisted forever
            -> Recover             if anything fails: blacklisted for 60 s
      -> Explore                   PRIORITY 2
         -> ArmSurvey              MoveIt -> survey pose
         ResumeExplore             True -> /explore/resume
         Exploring                 RUNNING (if explore_lite finishes, the robot stays put)
```

## Files

| File | Content |
|---|---|
| `silvanus_bt/tracking.py` | Pure logic, no ROS: bottle tracking, target choice, approach pose (tested) |
| `silvanus_bt/behaviours.py` | The tree's leaves |
| `silvanus_bt/tree.py` | Tree assembly and `main` |
| `silvanus_bt/config.py` | All parameters and their defaults |
| `config/bt_robot.yaml` | Values for the real robot |

Inputs: `/bottles/detections_3d` (bottle_localizer: positions + YOLO confidence), TF, `/map`.
Outputs: `navigate_to_pose` (Nav2), `/move_action` (MoveIt), `/pick_bottle`
(`silvanus_interfaces/action/PickBottle`), `/explore/resume` (explore_lite).
