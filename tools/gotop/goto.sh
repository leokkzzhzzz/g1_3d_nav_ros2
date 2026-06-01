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

# ── Preflight: localization + Nav2 must both be up ─────────────────
# capture only needs map->body TF; goto additionally needs the
# /navigate_to_pose action server, which is hosted by bt_navigator.
# Without it the script times out with "is nav2_launch.sh running?".
# Catch both cases here with a precise redirect.
preflight_fail() {
    echo "" >&2
    echo "ERROR: $1" >&2
    echo "" >&2
    echo "$2" >&2
    exit 1
}
pgrep -f rmw_zenohd          >/dev/null || preflight_fail \
    "rmw_zenohd not running." \
    "Run launch.sh first:
  docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/nav/launch.sh"
pgrep -f fastlio_mapping     >/dev/null || preflight_fail \
    "fast_lio not running." \
    "Run launch.sh and wait for 'ALL 6 NODES RUNNING'."
pgrep -f global_localization >/dev/null || preflight_fail \
    "open3d_loc not running (no map frame)." \
    "Run launch.sh and wait for 'ALL 6 NODES RUNNING'."
pgrep -f bt_navigator        >/dev/null || preflight_fail \
    "Nav2 not running (no /navigate_to_pose action server)." \
    "Run nav2_launch.sh after launch.sh:
  docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/nav/nav2_launch.sh"

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
