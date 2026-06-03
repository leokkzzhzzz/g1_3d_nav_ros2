#!/bin/bash
# launch_rgb.sh — G1 RGB nav runtime stack (localization only, Phase 1).
#
# Starts: zenoh router + LiDAR + D435i + FAST-LIVO2 + open3d_loc (ColoredICP)
# Nav2 and pointcloud_to_laserscan are out of scope for Phase 1.
#
# Usage (inside 3d_nav_rgb container):
#   bash /g1_3d_nav_rgb/tools/nav/launch_rgb.sh
set +e

if [ "${ENV:-auto}" = "auto" ]; then
    [ -f /.dockerenv ] && ENV=docker || ENV=native
fi
case $ENV in
    docker) WS_LIVOX=/root/3d_nav_g1/livox_ws; WS_G1=/root/3d_nav_g1/g1_ws; MAPS=/g1_3d_nav_rgb/maps ;;
    native) WS_LIVOX=$HOME/livox_ws;          WS_G1=$HOME/g1_ws;      MAPS=$HOME/g1_3d_nav_rgb/maps ;;
esac

echo "=== G1 RGB Nav ($ENV, FAST-LIVO2 + ColoredICP) ==="
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
echo -n "[1/5] rmw_zenohd :7448 ... "
ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/0.0.0.0:7448"];scouting/multicast/enabled=true' \
    ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/zenohd_rgb.log 2>&1 &
wait_for "rmw_zenohd" "grep -q 'Started Zenoh router' /tmp/zenohd_rgb.log" 15 \
    /tmp/zenohd_rgb.log "port 7448 already in use? pkill -9 -f rmw_zenohd"
echo "OK"

# ── 2. LiDAR ───────────────────────────────────────
echo -n "[2/5] LiDAR Driver (MID360) ... "
ros2 launch livox_ros_driver2 msg_MID360_launch.py > /tmp/lidar_rgb.log 2>&1 &
wait_for "LiDAR" "grep -q 'successfully enable' /tmp/lidar_rgb.log" 20 \
    /tmp/lidar_rgb.log "LiDAR off? ping 192.168.123.120? host_ip in MID360_config.json?"
echo "OK"

# ── 3. D435i Camera ────────────────────────────────
echo -n "[3/5] D435i Camera ... "
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true enable_depth:=false \
    color_fps:=15 color_width:=640 color_height:=480 > /tmp/camera_rgb.log 2>&1 &
wait_for "D435i" "ros2 topic info /camera/color/image_raw 2>/dev/null | grep -q sensor_msgs" 20 \
    /tmp/camera_rgb.log "D435i USB connected? realsense2_camera installed?"
echo "OK"

# ── 4. FAST-LIVO2 ──────────────────────────────────
echo -n "[4/5] FAST-LIVO2 ... "
ros2 launch fast_livo mapping_mid360.launch.py use_rviz:=False > /tmp/fastlivo2_rgb.log 2>&1 &
wait_for "aft_mapped_to_init" \
    "timeout 2 ros2 topic echo /aft_mapped_to_init --once 2>/dev/null | grep -q frame_id" 60 \
    /tmp/fastlivo2_rgb.log "no odometry? LiDAR+camera flowing? extrinsics in mid_360.yaml correct?"
echo "Odometry flowing"

# ── 5. open3d_loc (ColoredICP) ─────────────────────
echo -n "[5/5] open3d_loc (ColoredICP) ... "
if [ ! -f "$MAPS/scans.pcd" ]; then
    echo "FAIL: $MAPS/scans.pcd not found — run mapping first." >&2
    exit 1
fi
ros2 launch open3d_loc open3d_loc_g1.launch.py \
    map_file:="$MAPS/scans.pcd" > /tmp/loc_rgb.log 2>&1 &
wait_for "open3d_loc" "ros2 node list 2>/dev/null | grep -q global_localization" 60 \
    /tmp/loc_rgb.log "stuck waiting for /aft_mapped_to_init or /cloud_registered? FAST-LIVO2 running?"
echo "OK"

echo ""
echo "=== RGB NAV STACK READY (5 nodes) ==="
echo "rmw_zenohd → LiDAR → D435i → FAST-LIVO2 → open3d_loc(ColoredICP)"
echo ""
echo "Verify: ros2 topic hz /localization_3d"
echo "Map:    $MAPS/scans.pcd"
echo ""

wait
