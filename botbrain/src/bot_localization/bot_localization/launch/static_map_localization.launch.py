#!/usr/bin/env python3
"""
Static Map Localization Launch File
Uses map_server + AMCL for localization with .pgm/.yaml maps
Alternative to RTAB-Map for pre-built static maps
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    # Get workspace and config paths
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(launch_dir)))))
    config_file = os.path.join(workspace_dir, 'robot_config.yaml')
    
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)['robot_configuration']
    
    robot_name = config['robot_name']
    robot_model = config['robot_model']
    prefix = robot_name + '/' if robot_name != '' else ''
    base_frame = f'{prefix}base_footprint' if robot_model == 'g1' else f'{prefix}base_link'
    
    # Get default map or use argument
    default_map_name = config.get('default_static_map', 'map.yaml')
    
    # Map file path: {robot_model}_pkg/maps/{map_name}
    robot_pkg_share = get_package_share_directory(f'{robot_model}_pkg')
    default_map_path = os.path.join(workspace_dir, 'src', f'{robot_model}_pkg', 'maps', default_map_name)
    
    # Launch arguments
    map_yaml_file_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time'
    )
    
    # Get launch configurations
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # AMCL parameters file path
    # Try robot-specific config first, then fall back to bot_localization config
    try:
        robot_pkg_config = os.path.join(
            get_package_share_directory(f'{robot_model}_pkg'),
            'config',
            'amcl_params.yaml'
        )
        if os.path.exists(robot_pkg_config):
            amcl_params_file = robot_pkg_config
        else:
            # Use source directory path since install may not have the file
            amcl_params_file = os.path.join(
                workspace_dir,
                'src',
                'bot_localization',
                'bot_localization',
                'config',
                'amcl_params.yaml'
            )
    except:
        # Fallback to source directory
        amcl_params_file = os.path.join(
            workspace_dir,
            'src',
            'bot_localization',
            'bot_localization',
            'config',
            'amcl_params.yaml'
        )
    
    # Map Server Node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file,
            'topic_name': 'map',
            'frame_id': f'{prefix}map'
        }]
    )
    
    # AMCL Node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace=robot_name,
        output='screen',
        parameters=[amcl_params_file, {
            'use_sim_time': use_sim_time,
            'base_frame_id': base_frame,
            'odom_frame_id': f'{prefix}odom',
            'global_frame_id': f'{prefix}map',
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'set_initial_pose': False,
        }],
        remappings=[
            ('scan', 'scan'),
        ]
    )
    
    # Lifecycle Manager for map_server and amcl
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )
    
    # Optional: Include realsense launch if using depth cameras for scan
    # Uncomment if you want to use depth_to_laserscan
    # realsense_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_dir, 'realsense.launch.py')
    #     )
    # )
    
    return LaunchDescription([
        map_yaml_file_arg,
        use_sim_time_arg,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        # realsense_launch,  # Uncomment if needed
    ])
