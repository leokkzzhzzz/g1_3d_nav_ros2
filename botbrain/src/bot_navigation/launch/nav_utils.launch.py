import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(launch_dir)))))
    config_file = os.path.join(workspace_dir, 'robot_config.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)['robot_configuration']

    robot_name = config['robot_name']

    nav2_utils_node = Node(
        package='bot_navigation',
        executable='nav2_utils.py',
        name='nav2_utils',
        namespace=robot_name,
        output='screen',
    )

    goal_pose_bridge_node = Node(
        package='bot_navigation',
        executable='goal_pose_bridge.py',
        name='goal_pose_bridge',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'default_goal_frame': f'{robot_name}/map' if robot_name else 'map'
        }],
    )

    return LaunchDescription([
        nav2_utils_node,
        goal_pose_bridge_node,
    ])
