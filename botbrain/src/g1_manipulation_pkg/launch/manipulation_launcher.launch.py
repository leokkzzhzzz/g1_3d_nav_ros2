#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Launch file for g1_manipulation_pkg in BotBrain.

Launches:
  1. arm_controller      — real-time arm IK + DDS writer  (Mode-B: stops nav)
  2. dx3_hand_controller — dexterous hand DDS writer
  3. interactive_marker   — RViz 6-DOF goal markers (optional, load if display)

Reads ``robot_config.yaml`` for robot_name & network_interface.
"""

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ----- workspace & robot config -----
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(launch_dir))))
    )
    config_file = os.path.join(workspace_dir, "robot_config.yaml")
    try:
        with open(config_file, "r") as f:
            config = yaml.safe_load(f)["robot_configuration"]
    except Exception:
        config = {}

    robot_name = config.get("robot_name", "g1_robot")
    interface  = config.get("network_interface", "eth0")

    pkg_share = get_package_share_directory("g1_manipulation_pkg")
    params_file = os.path.join(pkg_share, "config", "manipulation_config.yaml")

    # ----- declare arguments -----
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("use_robot", default_value="true"))
    ld.add_action(DeclareLaunchArgument("interface", default_value=interface))
    ld.add_action(DeclareLaunchArgument("launch_markers", default_value="true"))

    common_params = [
        params_file,
        {
            "use_robot": LaunchConfiguration("use_robot"),
            "interface": LaunchConfiguration("interface"),
        },
    ]

    # ----- Arm Controller -----
    arm_controller = LaunchNode(
        package="g1_manipulation_pkg",
        executable="arm_controller",
        name="arm_controller",
        namespace=robot_name,
        output="screen",
        parameters=common_params,
    )
    ld.add_action(arm_controller)

    # ----- DX3 Hand Controller -----
    dx3_controller = LaunchNode(
        package="g1_manipulation_pkg",
        executable="dx3_controller",
        name="dx3_hand_controller",
        namespace=robot_name,
        output="screen",
        parameters=common_params,
    )
    ld.add_action(dx3_controller)

    # ----- Interactive Markers (optional, useful for RViz teleoperation) -----
    interactive_marker = LaunchNode(
        package="g1_manipulation_pkg",
        executable="interactive_marker",
        name="multi_ee_goal_markers",
        namespace=robot_name,
        output="screen",
        parameters=[
            params_file,
            {
                "fixed_frame": "pelvis",
                "right_topic": "manipulation/hand_goal/right",
                "left_topic":  "manipulation/hand_goal/left",
            },
        ],
    )
    ld.add_action(interactive_marker)

    return ld
