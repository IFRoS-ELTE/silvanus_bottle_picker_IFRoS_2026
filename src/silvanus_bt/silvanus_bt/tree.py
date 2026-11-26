"""Silvanus behaviour tree: explore and, when a bottle is seen, go and collect it.

    ros2 launch silvanus_bt bt_robot.launch.py
    py-trees-tree-viewer          # (another terminal) watch the tree live

Structure (-> Sequence, ? Selector, => Parallel; [m] = with memory):

 => Silvanus
    Bottles2BB                         sensors -> blackboard (always SUCCESS)
    ? Tasks                            priorities, no memory: re-evaluated every tick
       -> CollectBottle [m]            PRIORITY 1
          BottleToCollect?             condition: picks the most confident bottle
          ? Handle [m]
             -> Attempt [m]
                PauseExplore
                ? Approach [m]         (only with refine) drive to approach_distance
                ? Standoff [m]         refine: standoff_distance, facing the bottle
                -> ArmNear             arm to 'near'
                Pick                   pick_mode: action (/pick_bottle) | wait | skip
                MarkCollected          blacklisted forever
             -> Recover [m]            if anything in Attempt fails
                MarkFailed             blacklisted for retry_after s
       -> Explore [m]                  PRIORITY 2 (default)
          -> ArmSurvey                 arm to 'survey' (camera looking far)
          ResumeExplore
          Exploring                    RUNNING forever (if explore_lite finishes, it stays put)
"""

from __future__ import annotations

import py_trees
import py_trees_ros
import rclpy
from moveit_msgs.action import MoveGroup
from nav2_msgs.action import NavigateToPose
from py_trees.composites import Parallel, Selector, Sequence
from rclpy.executors import ExternalShutdownException

from silvanus_bt.behaviours import (BottlesToBB, ComputeApproachGoal, MarkTarget,
                                    SelectTarget, SetArmGoal, SetExplore, TargetWithin,
                                    WaitForPick)
from silvanus_bt.config import Config


def arm_to(cfg: Config, pose: str) -> py_trees.behaviour.Behaviour:
    """Subtree: move the arm to a named pose (or nothing with use_arm:=false)."""
    name = 'Arm' + pose.capitalize()
    if not cfg.use_arm:
        return py_trees.behaviours.Success(name=f'{name}(off)')
    return Sequence(name=name, memory=True, children=[
        SetArmGoal(cfg, pose, name=f'Set{pose.capitalize()}Goal'),
        py_trees_ros.action_clients.FromBlackboard(
            name=f'MoveIt:{pose}', action_type=MoveGroup, action_name='/move_action',
            key='arm_goal'),
    ])


def drive_to(cfg: Config, distance: float, label: str) -> py_trees.behaviour.Behaviour:
    """Subtree: if already within `distance` (+margin) of the bottle do nothing; otherwise
    compute the pose and send it to Nav2."""
    go = Sequence(name=f'Go{label}', memory=True, children=[
        ComputeApproachGoal(cfg, distance, name=f'Compute{label}Goal'),
        py_trees_ros.action_clients.FromBlackboard(
            name=f'Nav2:{label}', action_type=NavigateToPose, action_name=cfg.nav_action,
            key='nav_goal'),
    ])
    return Selector(name=label, memory=True, children=[
        TargetWithin(cfg, distance + cfg.already_there_extra, name=f'Within{label}?'),
        go,
    ])


def pick(cfg: Config) -> py_trees.behaviour.Behaviour:
    """Leaf: pick the bottle in front of the robot."""
    if cfg.pick_mode != 'action':
        return WaitForPick(cfg, name='WaitForPick' if cfg.pick_mode == 'wait' else 'Pick(skip)')
    # Imported here so the 'wait' / 'skip' modes do not need silvanus_interfaces
    from silvanus_interfaces.action import PickBottle
    goal = PickBottle.Goal()
    goal.max_attempts = int(cfg.pick_max_attempts)
    goal.bottle_timeout = float(cfg.pick_bottle_timeout)
    return py_trees_ros.action_clients.FromConstant(
        name='PickBottle', action_type=PickBottle, action_name=cfg.pick_action,
        action_goal=goal,
        generate_feedback_message=lambda msg: msg.feedback.stage)


def create_root(cfg: Config) -> py_trees.behaviour.Behaviour:
    attempt = [SetExplore(cfg, resume=False, name='PauseExplore')]
    if cfg.refine and cfg.approach_distance > cfg.standoff_distance:
        attempt.append(drive_to(cfg, cfg.approach_distance, 'Approach'))
    attempt += [
        drive_to(cfg, cfg.standoff_distance, 'Standoff'),
        arm_to(cfg, 'near'),
        pick(cfg),
        MarkTarget(cfg, permanent=True, name='MarkCollected'),
    ]

    collect = Sequence(name='CollectBottle', memory=True, children=[
        SelectTarget(cfg),
        Selector(name='Handle', memory=True, children=[
            Sequence(name='Attempt', memory=True, children=attempt),
            Sequence(name='Recover', memory=True, children=[
                MarkTarget(cfg, permanent=False, name='MarkFailed'),
            ]),
        ]),
    ])

    explore = Sequence(name='Explore', memory=True, children=[
        arm_to(cfg, 'survey'),
        SetExplore(cfg, resume=True, name='ResumeExplore'),
        py_trees.behaviours.Running(name='Exploring'),
    ])

    tasks = Selector(name='Tasks', memory=False, children=[collect, explore])
    return Parallel(
        name='Silvanus',
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
        children=[BottlesToBB(cfg), tasks])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = rclpy.create_node('silvanus_bt')
    cfg = Config.from_node(node)
    root = create_root(cfg)
    tree = py_trees_ros.trees.BehaviourTree(root=root, unicode_tree_debug=False)
    try:
        # No time limit: if a server is missing (Nav2, MoveIt, /pick_bottle) it keeps
        # warning 'waiting for action server ...' until it appears.
        tree.setup(node=node, timeout=py_trees.common.Duration.INFINITE)
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f'Tree setup failed: {exc}')
        tree.shutdown()
        rclpy.try_shutdown()
        return
    node.get_logger().info('Tree ready:\n' + py_trees.display.unicode_tree(root))
    tree.tick_tock(period_ms=1000.0 / max(cfg.tick_hz, 0.1))
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
