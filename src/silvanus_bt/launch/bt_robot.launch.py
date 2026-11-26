"""Starts the Silvanus behaviour tree with the robot parameters.

    ros2 launch silvanus_bt bt_robot.launch.py                       # fully automatic
    ros2 launch silvanus_bt bt_robot.launch.py pick_mode:=skip       # navigation only
    ros2 launch silvanus_bt bt_robot.launch.py pick_mode:=wait       # manual pick
    ros2 launch silvanus_bt bt_robot.launch.py use_arm:=false use_explore:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('silvanus_bt'), 'config', 'bt_robot.yaml')
    overrides = {
        'pick_mode': 'action',
        'use_arm': 'true',
        'use_explore': 'true',
        'selection': 'confidence',
    }
    args = [DeclareLaunchArgument('params_file', default_value=params)]
    args += [DeclareLaunchArgument(k, default_value=v) for k, v in overrides.items()]
    bt = Node(
        package='silvanus_bt',
        executable='silvanus_bt',
        name='silvanus_bt',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file'),
                    {k: LaunchConfiguration(k) for k in overrides}],
    )
    return LaunchDescription(args + [bt])
