#!/bin/bash
# mapping_launch_rgb.sh — FAST-LIVO2 RGB mapping stack for 3d_nav_rgb.
#
# Starts: zenoh router + LiDAR driver + D435i camera + FAST-LIVO2
# Stop with Ctrl+C — triggers mapping_save_rgb.sh to dump maps/scans.pcd.
#
# Usage (inside 3d_nav_rgb container):
#   bash /g1_3d_nav_rgb/tools/mapping/mapping_launch_rgb.sh
set +e
set -m

# ── 环境 ───────────────────────────────────────────
if [ "${ENV:-auto}" = "auto" ]; then
    [ -f /.dockerenv ] && ENV=docker || ENV=native
fi
case $ENV in
    docker) WS_LIVOX=/root/3d_nav_g1/livox_ws; WS_G1=/root/3d_nav_g1/g1_ws ;;
    native) WS_LIVOX=$HOME/livox_ws;          WS_G1=$HOME/g1_ws ;;
esac

echo "=== G1 RGB Mapping ($ENV, FAST-LIVO2 + D435i) ==="
echo "Cleaning SHM..."
rm -f /dev/shm/fastrtps_port* /dev/shm/fastrtps_* 2>/dev/null

echo "Killing old processes..."
pkill -9 -f zenoh_bridge_dds        2>/dev/null
pkill -9 -f rmw_zenohd              2>/dev/null
pkill -9 -f fastlivo_mapping        2>/dev/null
pkill -9 -f global_localization     2>/dev/null
pkill -9 -f livox                   2>/dev/null
pkill -9 -f realsense2_camera       2>/dev/null
sleep 2

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=30
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7448"]'

source /opt/ros/humble/setup.bash
source $WS_LIVOX/install/setup.bash
source $WS_G1/install/setup.bash
ros2 daemon stop 2>/dev/null; ros2 daemon start 2>/dev/null; sleep 1

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
    ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/zenohd_rgb.log 2>&1 &
PID_ZENOHD=$!
wait_for "rmw_zenohd" "grep -q 'Started Zenoh router' /tmp/zenohd_rgb.log" 15 \
    /tmp/zenohd_rgb.log "port 7448 already in use? pkill -9 -f rmw_zenohd"
echo "OK"

# ── 2. LiDAR ───────────────────────────────────────
echo -n "[2/4] LiDAR Driver (MID360) ... "
ros2 launch livox_ros_driver2 msg_MID360_launch.py > /tmp/lidar_rgb.log 2>&1 &
PID_LIDAR=$!
wait_for "LiDAR" "grep -q 'successfully enable' /tmp/lidar_rgb.log" 20 \
    /tmp/lidar_rgb.log "LiDAR off? ping 192.168.123.120? host_ip in MID360_config.json?"
echo "OK"

# ── 3. D435i Camera ────────────────────────────────
echo -n "[3/4] D435i Camera ... "
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true enable_depth:=false \
    color_fps:=15 color_width:=640 color_height:=480 > /tmp/camera_rgb.log 2>&1 &
PID_CAMERA=$!
wait_for "D435i" "ros2 topic info /camera/color/image_raw 2>/dev/null | grep -q sensor_msgs" 20 \
    /tmp/camera_rgb.log "D435i USB connected? realsense2_camera package installed?"
echo "OK"

# ── 4. FAST-LIVO2 ──────────────────────────────────
echo -n "[4/4] FAST-LIVO2 ... "
ros2 launch fast_livo mapping_mid360.launch.py use_rviz:=False > /tmp/fastlivo2_rgb.log 2>&1 &
PID_LIVO2=$!
wait_for "aft_mapped_to_init" \
    "timeout 2 ros2 topic echo /aft_mapped_to_init --once 2>/dev/null | grep -q frame_id" 60 \
    /tmp/fastlivo2_rgb.log "no odometry? LiDAR+camera data flowing? Check extrinsics in mid_360.yaml"
echo "Odometry flowing"

echo ""
echo "=== RGB MAPPING STACK READY ==="
echo "rmw_zenohd → LiDAR → D435i → FAST-LIVO2"
echo "Published: /cloud_registered  /aft_mapped_to_init  /rgb_img"
echo ""
echo "Drive G1 to accumulate colored scan. Ctrl+C to stop AND save."
echo "  Output: /g1_3d_nav_rgb/maps/scans.pcd (colored)"
echo ""

SAVED=0
on_stop() {
    [ $SAVED -eq 1 ] && return
    SAVED=1
    echo ""
    echo "=== Ctrl+C received — running mapping_save_rgb.sh ==="
    /g1_3d_nav_rgb/tools/mapping/mapping_save_rgb.sh
    SAVE_RC=$?
    echo "=== shutting down RGB mapping stack ==="
    kill -INT  $PID_LIVO2   2>/dev/null
    kill -INT  $PID_CAMERA  2>/dev/null
    kill -INT  $PID_LIDAR   2>/dev/null
    kill -INT  $PID_ZENOHD  2>/dev/null
    sleep 2
    pkill -9 -f fastlivo_mapping   2>/dev/null
    pkill -9 -f realsense2_camera  2>/dev/null
    pkill -9 -f livox              2>/dev/null
    pkill -9 -f rmw_zenohd         2>/dev/null
    echo "=== RGB mapping stack stopped ==="
    if [ $SAVE_RC -eq 0 ]; then
        echo "RESULT: SUCCESS — /g1_3d_nav_rgb/maps/scans.pcd"
    else
        echo "RESULT: FAIL (rc=$SAVE_RC)"
    fi
    exit $SAVE_RC
}
trap on_stop INT TERM

wait
