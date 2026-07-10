import os
from datetime import datetime
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_share = get_package_share_directory('zed_bridge')
    recorder_config = os.path.join(pkg_share, 'config', 'recorder_config.yaml')
    
    now = datetime.now()
    
    # Base: ~/Documents/bags 
    base_dir = os.path.expanduser("~/Documents/bags")
    
    year_str = now.strftime("%Y")
    month_str = now.strftime("%m")
    day_str = now.strftime("%d")
    
    daily_path = os.path.join(base_dir, year_str, month_str, day_str)
    
    # Criar as pastas fisicamente se não existirem
    os.makedirs(daily_path, exist_ok=True)
    
    bag_name = f"bag_{now.strftime('%Y-%m-%d_%H-%M-%S')}"
    
    full_uri = os.path.join(daily_path, bag_name)

    print(f"--- CONFIGURAÇÃO DE GRAVAÇÃO ---")
    print(f"Diretoria de destino: {daily_path}")
    print(f"Nome da Bag: {bag_name}")
    print(f"--------------------------------")

    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            
            ComposableNode(
                package='zed_bridge',
                plugin='ZedBridge',
                name='zed_bridge',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),

            # B: Recorder
            ComposableNode(
                package='rosbag2_composable_recorder',
                plugin='rosbag2_composable_recorder::ComposableRecorder',
                name='recorder',
                parameters=[
                    recorder_config,       # Carrega os tópicos do YAML
                    {'bag_name': full_uri} # SOBRESCREVE o local de gravação com o nosso caminho dinâmico
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen',
    )

    # 4. Nó de Transformada Estática (que o seu superior adicionou)
    tf_node = Node(
        package="tf2_ros", 
        executable="static_transform_publisher", 
        arguments=["-0.5", "0", "0.95", "0", "0", "0", "base_footprint", "zed_camera_center"]
    )

    return LaunchDescription([
        container,
        tf_node
    ])