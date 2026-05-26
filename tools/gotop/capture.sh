#!/bin/bash
# capture.sh — wrapper around capture_waypoints.py.
#
# Default waypoints yaml is /g1_3d_nav_ros2/data/waypoints.yaml — this
# is the bind-mounted host repo working tree, so captures persist
# across container stop/start. Override with WAYPOINTS_YAML env var.
#
# Usage:
#   docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/gotop/capture.sh
#   docker exec -it 3d_nav_ros2 \
#       env WAYPOINTS_YAML=/some/other/path.yaml \
#       /g1_3d_nav_ros2/tools/gotop/capture.sh

source /opt/ros/humble/setup.bash
source /botbrain_ws/install/setup.bash

WP="${WAYPOINTS_YAML:-/g1_3d_nav_ros2/data/waypoints.yaml}"
mkdir -p "$(dirname "$WP")"

exec python3 /g1_3d_nav_ros2/tools/gotop/capture_waypoints.py "$WP" "$@"
