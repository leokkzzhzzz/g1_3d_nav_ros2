#!/bin/bash
# mapping_record.sh — start grid_accumulator and hold the session.
#
# This is one half of the ROS2-native mapping toolchain. fast_lio is
# already running in mapping mode under launch.sh — it accumulates the
# 3D point cloud and writes scans.pcd via the /map_save service. This
# wrapper starts the 2D companion: grid_accumulator subscribes to
# /cloud_registered_body_1, classifies points by z, accumulates a 2D
# OccupancyGrid on /accumulated_grid.
#
# Run while operator drives G1 around to scan the workspace. Ctrl-C
# when done. Then call mapping_save.sh to dump both maps to disk.
#
# Pre-conditions:
#   - 3d_nav_ros2 container is up
#   - launch.sh has been run (fast_lio + lidar driver streaming
#     /cloud_registered_body_1, map->body TF flowing)
#
# Usage:
#   docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/mapping/mapping_record.sh

source /opt/ros/humble/setup.bash
source /botbrain_ws/install/setup.bash
exec python3 /g1_3d_nav_ros2/tools/mapping/grid_accumulator.py
