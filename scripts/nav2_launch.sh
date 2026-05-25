#!/bin/bash
# nav2_launch.sh — Nav2 + twist_mux startup for the merged 3d_nav_ros2 container
#
# Run AFTER launch.sh has finished (all 6 localization steps OK and rmw_zenohd
# is up on :7448). Brings up the botbrain Nav2 stack as part of the same
# Zenoh fabric.
#
# Topology:
#   nav2 controller_server -> /cmd_vel_nav (priority 10 input to twist_mux)
#   twist_mux -> /cmd_vel  (final controller output; consumed by g1_write_node
#                            in production, but g1_write_node is intentionally
#                            NOT started in this PoC).
#
# Out-of-scope for this script: g1_write_node. The robot will not move.

set +e

# ── Sanity ────────────────────────────────────────
if ! pgrep -f rmw_zenohd >/dev/null; then
    echo "ERROR: rmw_zenohd not running. Run launch.sh first." >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
source /botbrain_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=30
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7448"]'

# robot_config.yaml lives in /botbrain_ws (mounted from host); bot_navigation
# launch reads it at runtime.
[ -f /botbrain_ws/robot_config.yaml ] || {
    echo "ERROR: /botbrain_ws/robot_config.yaml missing. Bind-mount botbrain_ws." >&2
    exit 1
}

# ── helper ─────────────────────────────────────────
wait_for() { local desc=$1 cmd=$2 timeout=${3:-60}
    for i in $(seq 1 $timeout); do
        eval "$cmd" 2>/dev/null && return 0
        sleep 1
    done
    echo "  TIMEOUT: $desc"; return 1
}

# ── 1. Nav2 stack ─────────────────────────────────
echo -n "[1/2] Nav2 (controller + planner + bt_navigator + behaviors + smoother + waypoint_follower) ... "
ros2 launch bot_navigation navigation.launch.py > /tmp/nav2.log 2>&1 &
wait_for "Nav2 lifecycle active" \
    "ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active" 60
echo "OK"

# ── 2. twist_mux ──────────────────────────────────
echo -n "[2/2] twist_mux ... "
ros2 launch bot_bringup twist_mux.launch.py > /tmp/twist_mux.log 2>&1 &
wait_for "twist_mux node" "ros2 node list 2>/dev/null | grep -q twist_mux" 30
echo "OK"

echo ""
echo "=== Nav2 STACK RUNNING ==="
echo "  Verify (G1):  ros2 lifecycle get /controller_server   # → active"
echo "  Verify (Leo): rviz2 → set 2D Goal Pose → /plan + /cmd_vel_nav + /cmd_vel"
echo ""
echo "Session stays alive. Press Ctrl+C to stop."
wait
