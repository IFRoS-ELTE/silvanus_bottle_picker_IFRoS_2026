"""Bottle perception on the real Silvanus: detector, localizer and grasp_inference.

    ros2 launch grasp_pipeline robot.launch.py
    ros2 launch grasp_pipeline robot.launch.py device:=0        # YOLO on the first GPU

The robot's own bringup (silvanus-up.sh) runs the camera, arm and MoveIt; this starts no
hardware driver and no move_group. pick_bottle is not started here: it moves the arm, so it
is started by hand once this is running (pick-bottle in the Docker image).

The robot's D435 driver publishes under /camera/camera/ (namespace + camera name). It
publishes no point cloud, so grasp_inference builds one from the aligned depth.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    cam = LaunchConfiguration('camera')
    reliable = LaunchConfiguration('reliable')

    def topic(suffix):
        return PythonExpression(["'", cam, suffix, "'"])

    args = [
        DeclareLaunchArgument('camera', default_value='/camera/camera',
                              description='Namespace of the D435 topics'),
        DeclareLaunchArgument('model', default_value='yolo11n.pt',
                              description='YOLO weights (baked into the Docker image)'),
        DeclareLaunchArgument('device', default_value='cpu',
                              description="YOLO device: 'cpu', or '0' for the first GPU"),
        DeclareLaunchArgument('confidence', default_value='0.25'),
        # Reliable image subscriptions: needed over Wi-Fi, harmless on the robot itself
        DeclareLaunchArgument('reliable', default_value='true'),
        # The camera and bottle are still, and YOLO on the CPU is slow
        DeclareLaunchArgument('detection_max_age', default_value='5.0'),
    ]

    detector = Node(
        package='bottle_perception',
        executable='bottle_detector',
        name='bottle_detector',
        output='screen',
        parameters=[{
            'model': LaunchConfiguration('model'),
            'device': LaunchConfiguration('device'),
            'confidence': LaunchConfiguration('confidence'),
            'image_topic': topic('/color/image_raw'),
            'reliable': reliable,
        }],
    )
    localizer = Node(
        package='bottle_perception',
        executable='bottle_localizer',
        name='bottle_localizer',
        output='screen',
        parameters=[{
            'depth_topic': topic('/aligned_depth_to_color/image_raw'),
            'depth_info_topic': topic('/aligned_depth_to_color/camera_info'),
            'camera_info_topic': topic('/color/camera_info'),
            'target_frame': 'base_footprint',
            'reliable': reliable,
        }],
    )
    grasp = Node(
        package='grasp_pipeline',
        executable='grasp_inference',
        name='grasp_inference_client',
        output='screen',
        parameters=[{
            'use_graspnet': False,
            'depth_topic': topic('/aligned_depth_to_color/image_raw'),
            'reliable': reliable,
            'detection_max_age': LaunchConfiguration('detection_max_age'),
        }],
        remappings=[('/camera/color/camera_info', topic('/color/camera_info'))],
    )
    return LaunchDescription(args + [detector, localizer, grasp])
