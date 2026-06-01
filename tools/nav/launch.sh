#!/bin/bash
# G1 3D Nav — 统一启动入口 (Docker / Native 自适应)
# P3 path: 所有节点跑在 RMW=rmw_zenoh_cpp 之下,
# rmw_zenohd 在 :7448 作为 Zenoh router (host 端 RViz2 直连).
set +e  # 不因单步失败退出

# ── 环境 ───────────────────────────────────────────
if [ "${ENV:-auto}" = "auto" ]; then
    [ -f /.dockerenv ] && ENV=docker || ENV=native
fi
case $ENV in
    docker) WS_LIVOX=/root/3d_nav_g1/livox_ws; WS_G1=/root/3d_nav_g1/g1_ws; MAPS=/g1_3d_nav_ros2/maps ;;
    native) WS_LIVOX=$HOME/livox_ws;          WS_G1=$HOME/g1_ws;      MAPS=$HOME/g1_3d_nav/maps ;;
esac

# ── 清理 ───────────────────────────────────────────
echo "=== G1 3D Nav ($ENV, P3/rmw_zenoh_cpp) ==="
echo "Cleaning SHM..."
rm -f /dev/shm/fastrtps_port* /dev/shm/fastrtps_* 2>/dev/null

echo "Killing old processes..."
pkill -9 -f zenoh_bridge_dds 2>/dev/null
pkill -9 -f rmw_zenohd       2>/dev/null
pkill -9 -f fastlio_mapping 2>/dev/null
pkill -9 -f global_localization 2>/dev/null
pkill -9 -f livox 2>/dev/null
pkill -9 -f map_server 2>/dev/null
pkill -9 -f pointcloud_to_laserscan 2>/dev/null
sleep 2

# ── RMW = rmw_zenoh_cpp (所有 ros2 进程统一 Zenoh) ─
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=30
# 节点用 client 模式连本机 router :7448 (router 在 [1/6] 启)
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7448"]'

source /opt/ros/humble/setup.bash
source $WS_LIVOX/install/setup.bash
source $WS_G1/install/setup.bash
ros2 daemon stop 2>/dev/null; ros2 daemon start 2>/dev/null; sleep 1

# ── helper ─────────────────────────────────────────
# wait_for $desc $cmd [$timeout=60] [$logfile] [$hint]
#   On timeout, dumps tail of $logfile (if given) and prints $hint (if
#   given) so the operator sees the failing node's actual error inline,
#   not just "TIMEOUT". The script continues either way (set +e) so
#   downstream steps can still run their own diagnostics.
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

# ── 1. rmw_zenohd (Zenoh router 必须先起) ──────────
# Router 在 :7448 监听, 后续节点用 client 模式连入.
# host 端 RViz2 (RMW=rmw_zenoh_cpp) 直连 <G1 ip>:7448.
echo -n "[1/6] rmw_zenohd :7448 ... "
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/0.0.0.0:7448"];scouting/multicast/enabled=true' \
    ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/zenohd.log 2>&1 &
wait_for "rmw_zenohd" "grep -q 'Started Zenoh router' /tmp/zenohd.log" 15 \
    /tmp/zenohd.log "port 7448 already in use? Try: pkill -9 -f rmw_zenohd"
echo "OK"

# ── 2. LiDAR ───────────────────────────────────────
echo -n "[2/6] LiDAR Driver ... "
ros2 launch livox_ros_driver2 msg_MID360_launch.py > /tmp/lidar.log 2>&1 &
wait_for "LiDAR" "grep -q 'successfully enable' /tmp/lidar.log" 20 \
    /tmp/lidar.log "LiDAR off? Cable? 'ping 192.168.123.120'? host_ip in MID360_config.json matches host's IP on .123.x?"
echo "OK"

# ── 3. FAST-LIO ────────────────────────────────────
echo -n "[3/6] FAST-LIO ... "
ros2 launch fast_lio mapping.launch.py rviz:=false > /tmp/fastlio.log 2>&1 &
wait_for "Odometry" "timeout 2 ros2 topic echo /Odometry_loc --once 2>/dev/null | grep -q frame_id" 40 \
    /tmp/fastlio.log "no /livox/imu rate? -> LiDAR data not flowing despite driver ack. mid360.yaml extrinsic_T sane?"
echo "Odometry flowing"

# ── 4. open3d_loc ──────────────────────────────────
echo -n "[4/6] open3d_loc ... "
ros2 launch open3d_loc open3d_loc_g1.launch.py rviz:=false > /tmp/loc.log 2>&1 &
wait_for "open3d_loc" "ros2 node list 2>/dev/null | grep -q global_localization" 60 \
    /tmp/loc.log "stuck on 'Waiting for Odometry_loc'? D-003 QoS regression — check /Odometry_loc subscriber QoS matches FAST-LIO publisher (RELIABLE)"
echo "OK"

# ── 5. map_server ──────────────────────────────────
echo -n "[5/6] map_server ... "
ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:=$MAPS/accumulated_grid.yaml \
    -r __node:=map_server \
    -r /map:=/map_2d > /tmp/mapserver.log 2>&1 &
sleep 2
ros2 lifecycle set /map_server configure 2>/dev/null
ros2 lifecycle set /map_server activate 2>/dev/null
wait_for "map_2d" "ros2 topic info /map_2d 2>/dev/null | grep -q OccupancyGrid" 20 \
    /tmp/mapserver.log "$MAPS/accumulated_grid.yaml exists? Path correct? PGM file present alongside?"
echo "OK"

# ── 6. pointcloud_to_laserscan ─────────────────────
echo -n "[6/6] pointcloud_to_laserscan ... "
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
    -r __node:=pcl2scan \
    -p target_frame:=body \
    -p transform_tolerance:=0.01 \
    -p min_height:=-1.2 -p max_height:=0.15 \
    -p angle_min:=-3.1415926 -p angle_max:=3.1415926 -p angle_increment:=0.007 \
    -p scan_time:=0.1 \
    -p range_min:=0.2 -p range_max:=100.0 \
    -p use_inf:=true -p inf_epsilon:=1.0 \
    -p concurrency_level:=1 \
    -r /cloud_in:=/cloud_registered_body_1 > /tmp/laserscan.log 2>&1 &
wait_for "scan" "ros2 topic info /scan 2>/dev/null | grep -q LaserScan" 20 \
    /tmp/laserscan.log "/cloud_registered_body_1 publishing? Run: ros2 topic hz /cloud_registered_body_1"
echo "OK"

echo ""
echo "=== ALL 6 NODES RUNNING (RMW=rmw_zenoh_cpp) ==="
echo "rmw_zenohd:7448 → LiDAR → FAST-LIO → open3d_loc → map_server → laserscan"
echo ""
echo "Session stays alive. Press Ctrl+C to stop all."
echo "Verify (G1): ros2 topic hz /localization_3d"
echo "Verify (host): RMW=rmw_zenoh_cpp + connect tcp/<G1 ip>:7448 → ros2 topic list"

# 保持 daemon 活着, 数据可读
wait
