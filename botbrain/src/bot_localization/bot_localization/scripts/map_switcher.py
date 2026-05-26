#!/usr/bin/env python3
"""
Map Switcher Node - Runtime map switching for static maps
Allows switching between different .pgm/.yaml maps without restarting
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from std_srvs.srv import Empty
from nav2_msgs.srv import LoadMap
from ament_index_python.packages import get_package_share_directory
import yaml


class MapSwitcherNode(LifecycleNode):
    """
    Lifecycle node for switching static maps at runtime
    
    Services:
        ~/load_map (nav2_msgs/LoadMap) - Load a new map by filename
        ~/list_maps (std_srvs/Empty) - List available maps
    """
    
    def __init__(self):
        super().__init__('map_switcher')
        
        # Declare parameters
        self.declare_parameter('robot_model', '')
        self.declare_parameter('maps_directory', '')
        
        self.map_server_client = None
        self.load_map_service = None
        self.list_maps_service = None
        
        self.get_logger().info('Map Switcher initialized (UNCONFIGURED)')
    
    def on_configure(self, state: State):
        self.get_logger().info('Configuring Map Switcher...')
        
        # Get robot model
        robot_model = self.get_parameter('robot_model').value
        if not robot_model:
            self.get_logger().error('robot_model parameter not set')
            return TransitionCallbackReturn.FAILURE
        
        # Determine maps directory
        maps_dir = self.get_parameter('maps_directory').value
        if not maps_dir:
            # Default to {robot_model}_pkg/maps
            try:
                workspace_dir = os.path.abspath(
                    os.path.join(
                        get_package_share_directory(f'{robot_model}_pkg'),
                        '..', '..', '..', '..'
                    )
                )
                maps_dir = os.path.join(workspace_dir, 'src', f'{robot_model}_pkg', 'maps')
            except Exception as e:
                self.get_logger().error(f'Failed to determine maps directory: {e}')
                return TransitionCallbackReturn.FAILURE
        
        self.maps_directory = maps_dir
        self.get_logger().info(f'Maps directory: {self.maps_directory}')
        
        # Create service client for map_server
        self.map_server_client = self.create_client(
            LoadMap,
            'map_server/load_map'
        )
        
        self.get_logger().info('Configuration successful')
        return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self, state: State):
        self.get_logger().info('Activating Map Switcher...')
        
        # Create service servers
        self.load_map_service = self.create_service(
            LoadMap,
            'load_map',
            self.load_map_callback
        )
        
        self.list_maps_service = self.create_service(
            Empty,
            'list_maps',
            self.list_maps_callback
        )
        
        self.get_logger().info('Map Switcher active')
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State):
        self.get_logger().info('Deactivating Map Switcher...')
        
        if self.load_map_service:
            self.destroy_service(self.load_map_service)
        if self.list_maps_service:
            self.destroy_service(self.list_maps_service)
        
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: State):
        self.get_logger().info('Cleaning up Map Switcher...')
        
        if self.map_server_client:
            self.destroy_client(self.map_server_client)
        
        return TransitionCallbackReturn.SUCCESS
    
    def load_map_callback(self, request, response):
        """Load a new map by filename"""
        map_filename = request.map_url
        
        # Construct full path
        if not map_filename.endswith('.yaml'):
            map_filename += '.yaml'
        
        full_path = os.path.join(self.maps_directory, map_filename)
        
        if not os.path.exists(full_path):
            self.get_logger().error(f'Map file not found: {full_path}')
            response.result = LoadMap.Response.RESULT_MAP_DOES_NOT_EXIST
            return response
        
        # Call map_server's load_map service
        if not self.map_server_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('map_server/load_map service not available')
            response.result = LoadMap.Response.RESULT_INVALID_MAP_DATA
            return response
        
        map_request = LoadMap.Request()
        map_request.map_url = full_path
        
        future = self.map_server_client.call_async(map_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            response.result = future.result().result
            self.get_logger().info(f'Successfully loaded map: {map_filename}')
        else:
            response.result = LoadMap.Response.RESULT_INVALID_MAP_DATA
            self.get_logger().error(f'Failed to load map: {map_filename}')
        
        return response
    
    def list_maps_callback(self, request, response):
        """List all available .yaml map files"""
        try:
            yaml_files = [f for f in os.listdir(self.maps_directory) 
                         if f.endswith('.yaml')]
            self.get_logger().info(f'Available maps: {yaml_files}')
        except Exception as e:
            self.get_logger().error(f'Failed to list maps: {e}')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MapSwitcherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
