#!/usr/bin/env python3
"""
Map Switcher Launch File
Launches the map switcher node for runtime map switching
"""

import os
from launch import LaunchDescription
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
    
    # Map Switcher Node
    map_switcher_node = Node(
        package='bot_localization',
        executable='map_switcher.py',
        name='map_switcher',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'robot_model': robot_model,
            'maps_directory': '',  # Will be auto-determined
        }]
    )
    
    return LaunchDescription([
        map_switcher_node,
    ])
