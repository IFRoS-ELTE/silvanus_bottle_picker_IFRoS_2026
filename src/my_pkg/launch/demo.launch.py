"""Launch the read-only lidar watcher.

    ros2 launch my_pkg demo.launch.py

safe_drive is deliberately NOT launched here: anything that moves the robot
should be a deliberate, separate command with an operator at the e-stop.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='my_pkg',
            executable='scan_watch',
            name='scan_watch',
            output='screen',
        ),
    ])
