#!/usr/bin/env python3
"""Move the real xArm6 to a named joint pose (survey / near) via MoveIt.

  ros2 run grasp_pipeline arm_pose --ros-args \
    --params-file $(ros2 pkg prefix grasp_pipeline)/share/grasp_pipeline/config/arm_poses_robot.yaml \
    -p pose:=survey

  ros2 run grasp_pipeline arm_pose --ros-args ... -p pose:=near
"""

from __future__ import annotations

import math
import sys

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# Defaults match arm_poses_robot.yaml (degrees).
DEFAULT_SURVEY_DEG = [0.0, 0.0, -16.46, 0.0, -42.62, 0.0]
DEFAULT_NEAR_DEG = [-6.34, -16.30, -24.53, 1.46, 27.77, -9.76]


def deg_to_rad(values):
    return [math.radians(float(v)) for v in values]


class ArmPose(Node):
    def __init__(self) -> None:
        super().__init__('arm_pose')
        self.declare_parameter('pose', 'survey')  # survey | near
        self.declare_parameter('survey_joints_deg', DEFAULT_SURVEY_DEG)
        self.declare_parameter('near_joints_deg', DEFAULT_NEAR_DEG)
        self.declare_parameter('velocity_scaling', 0.1)
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('planning_attempts', 5)
        self.declare_parameter('group', 'xarm6')

        self.move_group = ActionClient(self, MoveGroup, '/move_action')
        self.vel = float(self.get_parameter('velocity_scaling').value)
        self.planning_time = float(self.get_parameter('planning_time').value)
        self.planning_attempts = int(self.get_parameter('planning_attempts').value)
        self.group = str(self.get_parameter('group').value)

    def joints_for(self, name: str):
        name = name.strip().lower()
        if name == 'survey':
            return deg_to_rad(self.get_parameter('survey_joints_deg').value)
        if name == 'near':
            return deg_to_rad(self.get_parameter('near_joints_deg').value)
        raise ValueError(f'Unknown pose {name!r}; use survey or near')

    def wait_moveit(self) -> None:
        self.get_logger().info('Waiting for /move_action ...')
        while rclpy.ok() and not self.move_group.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('... still waiting for /move_action',
                                   throttle_duration_sec=5.0)

    def move_to_joints(self, positions, what: str) -> None:
        c = Constraints()
        for joint_name, value in zip(ARM_JOINTS, positions):
            c.joint_constraints.append(JointConstraint(
                joint_name=joint_name, position=float(value),
                tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.group
        req.num_planning_attempts = self.planning_attempts
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.vel
        req.max_acceleration_scaling_factor = self.vel
        req.start_state.is_diff = True
        req.goal_constraints.append(c)
        goal.planning_options.plan_only = False
        self.get_logger().info(f'Planning: {what}')
        send = self.move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f'{what}: MoveIt goal rejected')
        result_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_fut)
        result = result_fut.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'{what}: MoveIt error {result.error_code.val}')
        self.get_logger().info(f'Done: {what}')

    def run(self, pose_name: str) -> None:
        joints = self.joints_for(pose_name)
        self.wait_moveit()
        self.move_to_joints(joints, f'{pose_name} pose')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmPose()
    pose = str(node.get_parameter('pose').value)
    try:
        node.run(pose)
    except (KeyboardInterrupt, SystemExit):
        pass
    except Exception as err:  # noqa: BLE001
        node.get_logger().error(str(err))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(1)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
