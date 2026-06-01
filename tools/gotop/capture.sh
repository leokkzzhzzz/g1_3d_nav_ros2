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

# ── Preflight: localization stack must be up so map->body TF flows ──
# Without rmw_zenohd no ROS comm; without fast_lio + open3d_loc the
# capture script will time out on TF lookup with the equally-cryptic
# "TIMEOUT" message. Catch it here with a clear redirect to launch.sh.
preflight_fail() {
    echo "" >&2
    echo "ERROR: $1" >&2
    echo "" >&2
    echo "Run launch.sh first and wait for 'ALL 6 NODES RUNNING':" >&2
    echo "  docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/nav/launch.sh" >&2
    exit 1
}
pgrep -f rmw_zenohd          >/dev/null || preflight_fail "rmw_zenohd not running."
pgrep -f fastlio_mapping     >/dev/null || preflight_fail "fast_lio not running (no /Odometry stream)."
pgrep -f global_localization >/dev/null || preflight_fail "open3d_loc not running (no map->odom TF)."

source /opt/ros/humble/setup.bash
source /botbrain_ws/install/setup.bash

WP="${WAYPOINTS_YAML:-/g1_3d_nav_ros2/data/waypoints.yaml}"
mkdir -p "$(dirname "$WP")"

exec python3 /g1_3d_nav_ros2/tools/gotop/capture_waypoints.py "$WP" "$@"
