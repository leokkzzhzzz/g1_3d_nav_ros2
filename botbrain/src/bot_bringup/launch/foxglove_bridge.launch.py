import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    # 获取机器人配置
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(launch_dir)))))
    robot_config_file = os.path.join(workspace_dir, 'robot_config.yaml')
    with open(robot_config_file, 'r') as f:
        config = yaml.safe_load(f)['robot_configuration']
    
    robot_name = config['robot_name']

    # 获取 Foxglove 配置文件路径
    foxglove_config_file = os.path.join(
        get_package_share_directory('bot_bringup'),
        'config',
        'foxglove_bridge.yaml'
    )

    # Launch 参数
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8765',
        description='Port for Foxglove WebSocket server'
    )

    address_arg = DeclareLaunchArgument(
        'address',
        default_value='0.0.0.0',
        description='Address to bind Foxglove WebSocket server'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=foxglove_config_file,
        description='Path to Foxglove Bridge config file'
    )

    # Foxglove Bridge 节点
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        namespace=robot_name,
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'port': LaunchConfiguration('port'),
                'address': LaunchConfiguration('address'),
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        port_arg,
        address_arg,
        config_file_arg,
        foxglove_bridge,
    ])
