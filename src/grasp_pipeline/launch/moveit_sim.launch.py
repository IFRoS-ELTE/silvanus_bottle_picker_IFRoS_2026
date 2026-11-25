"""MoveIt move_group for the simulated Silvanus arm.

Run it next to `ros2 launch scout_xarm_gazebo sim.launch.py`:

    ros2 launch grasp_pipeline moveit_sim.launch.py              # move_group only
    ros2 launch grasp_pipeline moveit_sim.launch.py rviz:=true   # + RViz with the MoveIt plugin

The planning model is the sim's own URDF (scout base, arm mount, xArm6, xArm gripper),
so plans avoid the chassis. move_group drives the sim's existing xarm6_traj_controller;
it starts no controllers or robot_state_publisher of its own.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    share = get_package_share_directory('grasp_pipeline')
    # Same xacro and argument the sim's spawn_robot.launch.py uses, so the model matches
    description = os.path.join(get_package_share_directory('scout_xarm_description'),
                               'urdf', 'scout_xarm.urdf.xacro')
    sim_controllers = os.path.join(get_package_share_directory('scout_xarm_gazebo'),
                                   'config', 'ros2_controllers.yaml')
    joint_limits = os.path.join(get_package_share_directory('xarm_moveit_config'),
                                'config', 'xarm6', 'joint_limits.yaml')

    moveit_config = (
        MoveItConfigsBuilder('scout_xarm', package_name='grasp_pipeline')
        .robot_description(file_path=description,
                           mappings={'ros2_control_params': sim_controllers})
        .robot_description_semantic(file_path=os.path.join(share, 'config', 'scout_xarm_sim.srdf.xacro'))
        .robot_description_kinematics(file_path=os.path.join(share, 'config', 'kinematics.yaml'))
        .joint_limits(file_path=joint_limits)
        .trajectory_execution(file_path=os.path.join(share, 'config', 'moveit_controllers.yaml'),
                              moveit_manage_controllers=False)
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
        .to_moveit_configs()
    )

    use_sim_time = {'use_sim_time': True}

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), use_sim_time],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('xarm_moveit_config'),
                                      'rviz', 'moveit.rviz')],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                    use_sim_time],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        move_group,
        rviz,
    ])
