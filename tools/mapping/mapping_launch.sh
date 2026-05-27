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
# equivalent. One window holds the session; another window runs
# tools/mapping/mapping_save.sh when you're ready to dump.
#
# Usage:
#   docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/mapping/mapping_launch.sh
#
# To stop: Ctrl+C in this terminal. Followed (typically) by
# /g1_3d_nav_ros2/tools/mapping/mapping_save.sh in another window
# before the Ctrl+C, otherwise the new map data dies with the process.
set +e

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
wait_for() { local desc=$1 cmd=$2 timeout=${3:-60}
    for i in $(seq 1 $timeout); do
        eval "$cmd" 2>/dev/null && return 0
        sleep 1
    done
    echo "  TIMEOUT: $desc"; return 1
}

# ── 1. rmw_zenohd ──────────────────────────────────
echo -n "[1/4] rmw_zenohd :7448 ... "
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/0.0.0.0:7448"];scouting/multicast/enabled=true' \
    ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/zenohd.log 2>&1 &
wait_for "rmw_zenohd" "grep -q 'Started Zenoh router' /tmp/zenohd.log" 15
echo "OK"

# ── 2. LiDAR ───────────────────────────────────────
echo -n "[2/4] LiDAR Driver ... "
ros2 launch livox_ros_driver2 msg_MID360_launch.py > /tmp/lidar.log 2>&1 &
wait_for "LiDAR" "grep -q 'successfully enable' /tmp/lidar.log" 20
echo "OK"

# ── 3. FAST-LIO (mapping mode) ─────────────────────
# pcd_save_en is true in mid360.yaml, so /map_save service will dump
# the accumulated cloud to ./test.pcd (cwd = /root) when called.
echo -n "[3/4] FAST-LIO (mapping mode) ... "
ros2 launch fast_lio mapping.launch.py rviz:=false > /tmp/fastlio.log 2>&1 &
wait_for "Odometry" "timeout 2 ros2 topic echo /Odometry_loc --once 2>/dev/null | grep -q frame_id" 40
echo "Odometry flowing"

# ── 4. grid_accumulator (2D OccupancyGrid) ─────────
echo -n "[4/4] grid_accumulator ... "
python3 /g1_3d_nav_ros2/tools/mapping/grid_accumulator.py > /tmp/grid.log 2>&1 &
wait_for "/accumulated_grid" "ros2 topic info /accumulated_grid 2>/dev/null | grep -q OccupancyGrid" 20
echo "OK"

echo ""
echo "=== MAPPING STACK READY (4 nodes, mapping mode) ==="
echo "rmw_zenohd:7448 → LiDAR → FAST-LIO mapping → grid_accumulator"
echo ""
echo "Drive G1 around the workspace to accumulate data."
echo "When done, in another window:"
echo "  docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/mapping/mapping_save.sh"
echo "It will dump scans.pcd + accumulated_grid.{pgm,yaml} into"
echo "/g1_3d_nav_ros2/maps/, back up the previous maps to .bak siblings."
echo ""
echo "Live grid stats: docker exec 3d_nav_ros2 tail -f /tmp/grid.log"
echo ""
echo "Session stays alive. Ctrl+C to stop all."

# 保持 daemon 活着, 数据可读
wait
