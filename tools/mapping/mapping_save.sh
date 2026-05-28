#!/bin/bash
# mapping_save.sh — dump 3D PCD + 2D PGM directly to /g1_3d_nav_ros2/maps/.
#
# Triggered either by mapping_launch.sh's SIGINT trap (one-shot save on
# Ctrl+C), or by a human in a separate window for an interim dump.
#
# fast_lio writes scans.pcd straight to /g1_3d_nav_ros2/maps/scans.pcd
# via mid360.yaml's `map_file_path` parameter — no test.pcd shuffle.
# map_saver_cli writes accumulated_grid.{pgm,yaml} on top of any existing
# files (no .bak; user opted for direct overwrite).
#
# Exit codes:
#   0  — PCD ok + 2D ok
#   1  — PCD save failed (2D not attempted)
#   2  — PCD ok but 2D save failed
#
# Pre-conditions:
#   - mapping_launch.sh running (fast_lio mapping mode + grid_accumulator)
#   - G1 driven around enough to cover the workspace
#
# Usage:
#   docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/mapping/mapping_save.sh

# set -u disabled around sourcing ROS env (AMENT_TRACE_SETUP_FILES is unbound under -u)

source /opt/ros/humble/setup.bash
source /botbrain_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7448"]'

MAPS=/g1_3d_nav_ros2/maps
PCD="$MAPS/scans.pcd"
PGM="$MAPS/accumulated_grid.pgm"
YML="$MAPS/accumulated_grid.yaml"
MIN_PCD_BYTES=$((1 * 1024 * 1024))   # 1 MB floor

mkdir -p "$MAPS"
T0=$(date +%s)

echo "[1/3] dumping 3D PCD via fast_lio /map_save ..."
ros2 service call /map_save std_srvs/srv/Trigger || true

# Validate by filesystem state, not service return value.
if [ ! -f "$PCD" ]; then
    echo "  FAIL: $PCD does not exist after /map_save." >&2
    exit 1
fi
PCD_SIZE=$(stat -c %s "$PCD" 2>/dev/null || echo 0)
PCD_MTIME=$(stat -c %Y "$PCD" 2>/dev/null || echo 0)
if [ "$PCD_SIZE" -lt "$MIN_PCD_BYTES" ]; then
    echo "  FAIL: $PCD size $PCD_SIZE < $MIN_PCD_BYTES (looks truncated)." >&2
    exit 1
fi
if [ "$PCD_MTIME" -lt "$T0" ]; then
    echo "  FAIL: $PCD mtime $PCD_MTIME older than save start $T0 (stale file)." >&2
    exit 1
fi
echo "  ok: $PCD ($PCD_SIZE bytes)"

echo "[2/3] dumping 2D PGM via map_saver_cli ..."
# map_saver_cli writes <prefix>.pgm + <prefix>.yaml. Point it straight at
# the final destination — no /tmp dance, no rename.
PREFIX="$MAPS/accumulated_grid"
if ! ros2 run nav2_map_server map_saver_cli \
        -t /accumulated_grid \
        --free 0.196 \
        --occ 0.65 \
        --mode trinary \
        -f "$PREFIX"; then
    echo "  FAIL: map_saver_cli returned non-zero." >&2
    exit 2
fi
if [ ! -f "$PGM" ] || [ ! -f "$YML" ]; then
    echo "  FAIL: map_saver_cli did not produce $PGM and $YML." >&2
    exit 2
fi
PGM_MTIME=$(stat -c %Y "$PGM" 2>/dev/null || echo 0)
if [ "$PGM_MTIME" -lt "$T0" ]; then
    echo "  FAIL: $PGM mtime older than save start $T0 (stale)." >&2
    exit 2
fi
echo "  ok: $PGM + $YML"

echo "[3/3] fixing yaml image path ..."
# map_saver_cli writes the image path relative to the yaml. Force the
# absolute container path so map_server picks it up from any cwd.
sed -i \
    -e "s|^image:.*|image: $PGM|" \
    -e "s|^free_thresh:.*|free_thresh: 0.196|" \
    -e "s|^occupied_thresh:.*|occupied_thresh: 0.65|" \
    -e "s|^mode:.*|mode: trinary|" \
    "$YML"
grep -q "^mode:"            "$YML" || echo "mode: trinary"        >> "$YML"
grep -q "^occupied_thresh:" "$YML" || echo "occupied_thresh: 0.65" >> "$YML"
grep -q "^free_thresh:"     "$YML" || echo "free_thresh: 0.196"   >> "$YML"

echo
echo "DONE. Files in $MAPS/ :"
ls -la "$PCD" "$PGM" "$YML" 2>&1 | sed 's|^| |'
exit 0
