from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # ZED ROS2 wrapper
        Node(
            package='zed_bridge',
            executable='zed_bridge',
            name='zed_bridge'
        ),
        # Static transform publisher
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.697", "0", "0.0", "0", "0", "0", "base_footprint", "zed_camera_center"]
        ),
        # ZED X stereo baseline: 120mm total, 60mm each side of center (Y-left convention)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0.06", "0", "0", "0", "0", "zed_camera_center", "zed_camera_left"]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "-0.06", "0", "0", "0", "0", "zed_camera_center", "zed_camera_right"]
        ),
    ])