"""Lanza el detector de botellas.

    ros2 launch bottle_perception detector.launch.py                 # simulacion
    ros2 launch bottle_perception detector.launch.py use_sim_time:=false confidence:=0.4
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='true en Gazebo, false en el robot real'),
        DeclareLaunchArgument('model', default_value='yolo11n.pt',
                              description='Pesos de YOLO (se descargan solos la primera vez)'),
        DeclareLaunchArgument('confidence', default_value='0.25',
                              description='Confianza minima (0-1) para aceptar una deteccion'),
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('reliable', default_value='false',
                              description='true over Wi-Fi (real robot): best effort loses big images'),
    ]
    detector = Node(
        package='bottle_perception',
        executable='bottle_detector',
        name='bottle_detector',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'model': LaunchConfiguration('model'),
            'confidence': LaunchConfiguration('confidence'),
            'image_topic': LaunchConfiguration('image_topic'),
            'reliable': LaunchConfiguration('reliable'),
        }],
    )
    return LaunchDescription(args + [detector])
