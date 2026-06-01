#!/bin/bash
# G1 Mapping — fast_lio + grid_accumulator only.
#
# Use this instead of tools/launch.sh when you need to (re)build the
# maps. Same first three steps as launch.sh — zenoh router, lidar
# driver, fast_lio in mapping mode — and then grid_accumulator
# producing the 2D OccupancyGrid in real time. open3d_loc / map_server /
# pointcloud_to_laserscan are NOT started, because:
#
#   - open3d_loc would try to ICP-match against the existing scans.pcd
#     while we're trying to build a new one — that's exactly the
#     pre-existing bug that made fitness drop to 0.0
#   - map_server would load the stale accumulated_grid.pgm; grid we're
#     about to replace anyway
#   - pcl2laserscan is for nav2's local costmap, not used during mapping
#
# Compared to ROS1's mapping flow (3 separate terminals: lidar +
# fast_lio + ground_cloud_accumulator), this is the all-in-one ROS2
# equivalent. Ctrl+C this terminal to stop AND save in one shot.
#
# Save contract (ADR-007):
#   - SIGINT here triggers mapping_save.sh.
#   - PCD save must succeed (file present, >= 1MB, mtime after Ctrl+C)
#     before the 2D PGM is dumped. If PCD fails, grid_accumulator is
#     SIGTERM'd (no 2D file) — all-or-nothing.
#
# Usage:
#   docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/mapping/mapping_launch.sh
set +e
set -m  # job control: each backgrounded child gets its own process group, so Ctrl+C in our terminal stays with mapping_launch.sh and does NOT broadcast to fast_lio / zenohd / lidar / grid_accumulator. The trap handler is the sole authority on killing them, in the order it chooses.

# ── 环境 ───────────────────────────────────────────
if [ "${ENV:-auto}" = "auto" ]; then
    [ -f /.dockerenv ] && ENV=docker || ENV=native
fi
case $ENV in
    docker) WS_LIVOX=/root/3d_nav_g1/livox_ws; WS_G1=/root/3d_nav_g1/g1_ws ;;
    native) WS_LIVOX=$HOME/livox_ws;          WS_G1=$HOME/g1_ws ;;
esac

# ── 清理 ───────────────────────────────────────────
echo "=== G1 Mapping ($ENV, fast_lio mapping mode + grid_accumulator) ==="
echo "Cleaning SHM..."
rm -f /dev/shm/fastrtps_port* /dev/shm/fastrtps_* 2>/dev/null

echo "Killing old processes..."
pkill -9 -f zenoh_bridge_dds         2>/dev/null
pkill -9 -f rmw_zenohd               2>/dev/null
pkill -9 -f fastlio_mapping          2>/dev/null
pkill -9 -f global_localization      2>/dev/null
pkill -9 -f livox                    2>/dev/null
pkill -9 -f map_server               2>/dev/null
pkill -9 -f pointcloud_to_laserscan  2>/dev/null
pkill -9 -f grid_accumulator         2>/dev/null
sleep 2

# ── RMW = rmw_zenoh_cpp ────────────────────────────
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=30
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7448"]'

source /opt/ros/humble/setup.bash
source $WS_LIVOX/install/setup.bash
source $WS_G1/install/setup.bash
ros2 daemon stop 2>/dev/null; ros2 daemon start 2>/dev/null; sleep 1

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

# ── 1. rmw_zenohd ──────────────────────────────────
echo -n "[1/4] rmw_zenohd :7448 ... "
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/0.0.0.0:7448"];scouting/multicast/enabled=true' \
    ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/zenohd.log 2>&1 &
PID_ZENOHD=$!
wait_for "rmw_zenohd" "grep -q 'Started Zenoh router' /tmp/zenohd.log" 15 \
    /tmp/zenohd.log "port 7448 already in use? Try: pkill -9 -f rmw_zenohd"
echo "OK"

# ── 2. LiDAR ───────────────────────────────────────
echo -n "[2/4] LiDAR Driver ... "
ros2 launch livox_ros_driver2 msg_MID360_launch.py > /tmp/lidar.log 2>&1 &
PID_LIDAR=$!
wait_for "LiDAR" "grep -q 'successfully enable' /tmp/lidar.log" 20 \
    /tmp/lidar.log "LiDAR off? Cable? 'ping 192.168.123.120'? host_ip in MID360_config.json matches host's IP on .123.x?"
echo "OK"

# ── 3. FAST-LIO (mapping mode) ─────────────────────
# mid360.yaml sets map_file_path=/g1_3d_nav_ros2/maps/scans.pcd so the
# /map_save service writes the PCD directly to the canonical location.
echo -n "[3/4] FAST-LIO (mapping mode) ... "
ros2 launch fast_lio mapping.launch.py rviz:=false > /tmp/fastlio.log 2>&1 &
PID_FASTLIO=$!
wait_for "Odometry" "timeout 2 ros2 topic echo /Odometry_loc --once 2>/dev/null | grep -q frame_id" 40 \
    /tmp/fastlio.log "no /livox/imu rate? -> LiDAR data not flowing despite driver ack. mid360.yaml extrinsic_T sane?"
echo "Odometry flowing"

# ── 4. grid_accumulator (2D OccupancyGrid) ─────────
# --map-frame camera_init: in mapping mode no node publishes the `map`
# frame (open3d_loc is intentionally not started — see preamble). The
# TF root is fast_lio's `camera_init`. Without this override,
# grid_accumulator's default `lookup_transform("map", "body")` raises
# forever and `frames` stays at 0.
echo -n "[4/4] grid_accumulator ... "
python3 /g1_3d_nav_ros2/tools/mapping/grid_accumulator.py \
    --map-frame camera_init > /tmp/grid.log 2>&1 &
PID_GRID=$!
if wait_for "/accumulated_grid" "ros2 topic info /accumulated_grid 2>/dev/null | grep -q OccupancyGrid" 30 \
    /tmp/grid.log "TF lookup 'camera_init -> body' failing? FAST-LIO not publishing TF? Python deps missing?"; then
    echo "OK"
else
    echo "FAIL: grid_accumulator did not register /accumulated_grid (see /tmp/grid.log)" >&2
fi

echo ""
echo "=== MAPPING STACK READY (4 nodes, mapping mode) ==="
echo "rmw_zenohd:7448 → LiDAR → FAST-LIO mapping → grid_accumulator"
echo ""
echo "Drive G1 around the workspace to accumulate data."
echo "Ctrl+C in THIS terminal to stop AND save in one shot:"
echo "  - PCD ok + 2D ok       → both files in /g1_3d_nav_ros2/maps/"
echo "  - PCD fail             → no 2D written (all-or-nothing)"
echo ""
echo "Live grid stats: docker exec 3d_nav_ros2 tail -f /tmp/grid.log"
echo ""

# ── SIGINT/SIGTERM trap: one-shot save ──────────────
# ADR-007 contract: PCD save is the gate. If PCD fails we SIGTERM
# grid_accumulator before it can write — magic of process timing here is
# that grid_accumulator.py doesn't write anything on shutdown anyway
# (map_saver_cli is the writer); we kill it just to prevent any future
# map_saver_cli call from this script.
SAVED=0
on_stop() {
    [ $SAVED -eq 1 ] && return
    SAVED=1
    echo ""
    echo "=== Ctrl+C received — running mapping_save.sh ==="
    /g1_3d_nav_ros2/tools/mapping/mapping_save.sh
    SAVE_RC=$?
    if [ $SAVE_RC -ne 0 ]; then
        echo "=== SAVE FAILED (rc=$SAVE_RC) — killing grid_accumulator without dumping 2D ==="
        kill -TERM $PID_GRID 2>/dev/null
    fi
    echo "=== shutting down mapping stack ==="
    # Order: grid → fastlio → lidar → zenohd. Reverse of startup.
    kill -INT  $PID_GRID    2>/dev/null
    kill -INT  $PID_FASTLIO 2>/dev/null
    kill -INT  $PID_LIDAR   2>/dev/null
    kill -INT  $PID_ZENOHD  2>/dev/null
    sleep 2
    pkill -9 -f grid_accumulator  2>/dev/null
    pkill -9 -f fastlio_mapping   2>/dev/null
    pkill -9 -f livox             2>/dev/null
    pkill -9 -f rmw_zenohd        2>/dev/null
    echo "=== mapping stack stopped ==="
    if [ $SAVE_RC -eq 0 ]; then
        echo "RESULT: SUCCESS — maps in /g1_3d_nav_ros2/maps/"
    else
        echo "RESULT: FAIL (rc=$SAVE_RC) — no usable map produced this run"
    fi
    exit $SAVE_RC
}
trap on_stop INT TERM

echo "Session stays alive. Ctrl+C to stop AND save."

# 保持 daemon 活着, 数据可读
wait
