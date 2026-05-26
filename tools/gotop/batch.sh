#!/bin/bash
# batch.sh — wrapper around navigate_batch.py.
#
# Default waypoints yaml is /g1_3d_nav_ros2/data/waypoints.yaml.
# Default report path is /g1_3d_nav_ros2/data/batch_report.md (each
# run overwrites). Override with WAYPOINTS_YAML / --output respectively.
#
# All extra args are forwarded — pass --labels / --all / --rounds /
# --shuffle the same way you would to the underlying python script.
#
# Usage:
#   docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/gotop/batch.sh --all --rounds 3 --shuffle
#   docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/gotop/batch.sh --labels kitchen,door1 --rounds 3
#   docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/gotop/batch.sh --labels @kitchen_zone --rounds 3

source /opt/ros/humble/setup.bash
source /botbrain_ws/install/setup.bash

WP="${WAYPOINTS_YAML:-/g1_3d_nav_ros2/data/waypoints.yaml}"
REPORT_DEFAULT=/g1_3d_nav_ros2/data/batch_report.md
mkdir -p "$(dirname "$WP")" "$(dirname "$REPORT_DEFAULT")"

case " $* " in
    *" --output "*) ;;
    *) set -- --output "$REPORT_DEFAULT" "$@" ;;
esac

exec python3 /g1_3d_nav_ros2/tools/gotop/navigate_batch.py "$WP" "$@"
