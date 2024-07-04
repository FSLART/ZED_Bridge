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
        Node(package="tf2_ros", executable="static_transform_publisher", arguments=["-0.5", "0", "0.95", "0", "0", "0", "base_footprint", "zed_camera_center"]),
    ])