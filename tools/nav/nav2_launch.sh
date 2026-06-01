#!/bin/bash
# nav2_launch.sh — Nav2 + twist_mux + g1_write_node startup for 3d_nav_ros2
#
# Run AFTER launch.sh has finished (all 6 localization steps OK and rmw_zenohd
# is up on :7448). Brings up the botbrain Nav2 stack + twist_mux + the SDK
# write node, all on the same Zenoh fabric.
#
# Topology:
#   nav2 controller_server -> /cmd_vel_nav  (priority 10 input to twist_mux)
#   nav2 behavior_server   -> /cmd_vel       (recovery channel; 4 publishers)
#   zero_vel_publisher     -> /cmd_vel_zero  (priority 1 fallback)
#   ...
#   twist_mux              -> /cmd_vel_out   (final controller output)
#   g1_write_node          -> SDK LocoClient.Move()  (the robot moves)
#
# Safety preconditions (operator's responsibility; see README Quick start):
#   - Operator on site, can see G1.
#   - >= 1 m clearance around G1; not on a ledge.
#   - RC controller in hand; L2+B available as the hardware brake.
#   - `ros2 service call /emergency_stop std_srvs/srv/SetBool "{}"` is the
#     in-stack fail-passive brake (toggle: first call ON, second OFF).
#
# ADRs: D-008 (merged container), D-009 (nav2_params topic forks),
#       D-011 v3 (botbrain default nav2_params.yaml is upstream-authoritative;
#                 motion default-on; no ENABLE_MOTION flag).

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

G1_WRITE_BIN=/botbrain_ws/install/g1_pkg/lib/g1_pkg/g1_write_node
[ -x "$G1_WRITE_BIN" ] || {
    echo "ERROR: $G1_WRITE_BIN not found / not executable. Build g1_pkg first." >&2
    exit 1
}

# ── helper ─────────────────────────────────────────
# wait_for $desc $cmd [$timeout=60] [$logfile] [$hint]
#   On timeout, dumps tail of $logfile (if given) inline so the failing
#   node's actual stderr is visible without a separate `tail` step.
wait_for() {
    local desc=$1 cmd=$2 timeout=${3:-60} logfile=${4:-} hint=${5:-}
    for i in $(seq 1 $timeout); do
        eval "$cmd" 2>/dev/null && return 0
        sleep 1
    done
    echo "  TIMEOUT after ${timeout}s: $desc" >&2
    if [ -n "$logfile" ] && [ -f "$logfile" ]; then
        echo "  ── tail of $logfile (last 25 lines) ──" >&2
        tail -25 "$logfile" 2>/dev/null | sed 's/^/  | /' >&2
        echo "  ───────────────────────────────────────────" >&2
    fi
    [ -n "$hint" ] && echo "  HINT: $hint" >&2
    return 1
}

# die $msg [$logfile] [$hint]
#   Hard-stop with the failing step's log tail dumped inline.
die() {
    local msg=$1 logfile=${2:-} hint=${3:-}
    echo "" >&2
    echo "FATAL: $msg" >&2
    if [ -n "$logfile" ] && [ -f "$logfile" ]; then
        echo "── tail of $logfile (last 30 lines) ──" >&2
        tail -30 "$logfile" 2>/dev/null | sed 's/^/  | /' >&2
        echo "──────────────────────────────────────────────" >&2
    fi
    [ -n "$hint" ] && echo "HINT: $hint" >&2
    exit 1
}

# ── 1/3 Nav2 stack ─────────────────────────────────
echo -n "[1/3] Nav2 (controller + planner + bt_navigator + behaviors + smoother + waypoint_follower) ... "
ros2 launch bot_navigation navigation.launch.py > /tmp/nav2.log 2>&1 &
wait_for "Nav2 lifecycle active" \
    "ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active" 60 \
    /tmp/nav2.log "MPPI param type mismatch (e.g. vy_max: 0 must be 0.0)? Or D-009 topic-fork yaml missing? Verify nav2_params.yaml and rebuild g1_pkg if needed." \
    || die "Nav2 lifecycle did not reach active." /tmp/nav2.log \
        "Common causes: (1) MPPI yaml int/double mismatch, (2) costmap subscribed to a topic that isn't publishing yet, (3) g1_pkg install stale (rebuild). 'colcon-symlink-install-trap' memory note may apply."
echo "OK"

# ── 2/3 twist_mux ──────────────────────────────────
echo -n "[2/3] twist_mux ... "
ros2 launch bot_bringup twist_mux.launch.py > /tmp/twist_mux.log 2>&1 &
wait_for "twist_mux node" \
    "ros2 node list 2>/dev/null | grep -q twist_mux" 30 \
    /tmp/twist_mux.log "twist_mux config yaml present and parseable?" \
    || die "twist_mux did not register." /tmp/twist_mux.log "twist_mux is bot_bringup's standard launch — failure usually means the bot_bringup install is broken or twist_mux yaml is malformed."
echo "OK"

# ── 2.5/3 zero_vel_publisher activate (R-001 fix) ──
# bot_bringup/twist_mux.launch.py registers OnProcessStart -> TRANSITION_CONFIGURE
# for zero_vel_publisher, but under rmw_zenoh_cpp the lifecycle event handler
# does not deliver. Without this manual step /cmd_vel_zero stays silent and
# /cmd_vel_out goes idle whenever Nav2 stops publishing /cmd_vel_nav.
echo -n "[2.5/3] zero_vel_publisher lifecycle activate (R-001 manual fix) ... "
wait_for "zero_vel_publisher node" \
    "ros2 node list 2>/dev/null | grep -q zero_vel_publisher" 15 \
    /tmp/twist_mux.log "zero_vel_publisher should have registered with twist_mux launch" \
    || die "zero_vel_publisher did not register." /tmp/twist_mux.log
ros2 lifecycle set /zero_vel_publisher configure >/dev/null 2>&1
ros2 lifecycle set /zero_vel_publisher activate  >/dev/null 2>&1
wait_for "zero_vel_publisher active" \
    "ros2 lifecycle get /zero_vel_publisher 2>/dev/null | grep -q active" 15 \
    /tmp/twist_mux.log "lifecycle transitions failed silently — check zero_vel_publisher.py for crash on configure" \
    || die "zero_vel_publisher did not reach active." /tmp/twist_mux.log
echo "OK"

# ── 3/3 g1_write_node (SDK control bridge) ─────────
# D-011 v3: motion is default-on. Safety relies on /emergency_stop service +
# RC controller + on-site operator. R-005 dead-man-switch is a separate
# production-deploy gate (not a test-time blocker).
echo -n "[3/3] g1_write_node (SDK Twist -> LocoClient.Move) ... "
nohup "$G1_WRITE_BIN" > /tmp/walk.log 2>&1 &
G1_WRITE_PID=$!
wait_for "robot_write_node registered" \
    "ros2 node list 2>/dev/null | grep -q robot_write_node" 15 \
    /tmp/walk.log "g1_write_node binary may be ABI-broken — rebuild g1_pkg" \
    || die "g1_write_node did not register." /tmp/walk.log
ros2 lifecycle set /robot_write_node configure >/dev/null 2>&1
wait_for "robot_write_node configured (SDK init done)" \
    "ros2 lifecycle get /robot_write_node 2>/dev/null | grep -q inactive" 30 \
    /tmp/walk.log "SDK init usually fails when (a) Unitree network 192.168.123.x not reachable, or (b) another instance of g1_write_node already holds the SDK channel — pkill first" \
    || die "g1_write_node configure failed." /tmp/walk.log
ros2 lifecycle set /robot_write_node activate  >/dev/null 2>&1
wait_for "robot_write_node active" \
    "ros2 lifecycle get /robot_write_node 2>/dev/null | grep -q active" 10 \
    /tmp/walk.log \
    || die "g1_write_node activate failed." /tmp/walk.log
echo "OK (PID $G1_WRITE_PID)"

echo ""
echo "=== STACK READY: G1 motion ENABLED ==="
echo "  /cmd_vel_out 是 final twist source；g1_write_node -> Unitree SDK LocoClient.Move"
echo ""
echo "Verify (in container):"
echo "  ros2 lifecycle get /robot_write_node    # active [3]"
echo "  ros2 topic info -v /cmd_vel_out         # subscriber count >= 1"
echo ""
echo "Send a goal (host RViz2):"
echo "  2D Pose Estimate (D-007 initial pose) -> 2D Goal Pose -> watch G1 walk"
echo ""
echo "Brake on demand:"
echo "  ros2 service call /emergency_stop std_srvs/srv/SetBool '{}'   # toggle"
echo ""
echo "Session stays alive. Ctrl+C to stop the whole stack."
wait
