import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Pegar o caminho do pacote zed_bridge
    pkg_share = get_package_share_directory('zed_bridge')
    
    # Caminho para o config do gravador
    recorder_config = os.path.join(pkg_share, 'config', 'recorder_config.yaml')

    # 1. O Contentor Composable (O "Escritório" partilhado)
    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            
            # A: A SUA ZedBridge (Agora carregada como plugin)
            ComposableNode(
                package='zed_bridge',
                plugin='ZedBridge',       # O nome que registamos no C++ e CMake
                name='zed_bridge',
                extra_arguments=[{'use_intra_process_comms': True}] # ZERO-COPY
            ),

            # B: O Gravador Composable
            ComposableNode(
                package='rosbag2_composable_recorder',
                plugin='rosbag2_composable_recorder::ComposableRecorder',
                name='recorder',
                parameters=[recorder_config],
                extra_arguments=[{'use_intra_process_comms': True}] # ZERO-COPY
            )
        ],
        output='screen',
    )

    # 2. O Nó de Ponte (Python)
    # Este script controla quando começar/parar a gravação
    bridge_node = Node(
        package='zed_bridge',
        executable='recording_bridge.py',
        name='recording_bridge',
        output='screen'
    )
    
    tf_node = Node(
        package="tf2_ros", 
        executable="static_transform_publisher", 
        arguments=["-0.5", "0", "0.95", "0", "0", "0", "base_footprint", "zed_camera_center"])

    return LaunchDescription([
        container,
        bridge_node,
        tf_node
    ])