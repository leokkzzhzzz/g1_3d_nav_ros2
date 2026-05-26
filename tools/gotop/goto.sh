#!/bin/bash
# goto.sh — wrapper around goto_waypoint.py.
#
# Default waypoints yaml is /g1_3d_nav_ros2/data/waypoints.yaml.
# Override with WAYPOINTS_YAML env var. The CSV history goes to
# /g1_3d_nav_ros2/data/goto_history.csv by default — append-only, so
# multiple sessions accumulate.
#
# Usage:
#   docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/gotop/goto.sh
#
# All extra args after the script name are forwarded to goto_waypoint.py
# — useful e.g. for `--csv /tmp/some_other_history.csv`.

source /opt/ros/humble/setup.bash
source /botbrain_ws/install/setup.bash

WP="${WAYPOINTS_YAML:-/g1_3d_nav_ros2/data/waypoints.yaml}"
CSV_DEFAULT=/g1_3d_nav_ros2/data/goto_history.csv
mkdir -p "$(dirname "$WP")" "$(dirname "$CSV_DEFAULT")"

# If the operator didn't pass --csv, slot in our persistent default.
case " $* " in
    *" --csv "*) ;;
    *) set -- --csv "$CSV_DEFAULT" "$@" ;;
esac

exec python3 /g1_3d_nav_ros2/tools/gotop/goto_waypoint.py "$WP" "$@"
