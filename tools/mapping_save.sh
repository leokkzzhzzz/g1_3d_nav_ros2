#!/bin/bash
# mapping_save.sh — dump 3D PCD + 2D PGM directly to /g1_3d_nav_ros2/maps/.
#
# This is the other half of the ROS2-native mapping toolchain. After
# mapping_record.sh has run for long enough that grid_accumulator has
# accumulated the workspace, this wrapper:
#
#   1. Asks fast_lio (already running in mapping mode) to dump the 3D
#      PCD via the /map_save service.
#   2. Asks nav2_map_server's map_saver_cli to dump the 2D OccupancyGrid
#      currently published on /accumulated_grid.
#   3. Backs up the previous maps to .bak siblings so the operator has
#      a one-step rollback (mv .bak back).
#   4. Moves both new maps into the mount target /g1_3d_nav_ros2/maps/
#      (= host side /home/unitree/g1_3d_nav_ros2_repo/maps/, the canon
#      repo working tree). git status on the host then shows the diff.
#   5. Fixes the yaml's image path so map_server picks up the new pgm
#      from the canonical mount location.
#
# Pre-conditions:
#   - launch.sh + mapping_record.sh both running (fast_lio mapping
#     active, grid_accumulator publishing /accumulated_grid)
#   - the operator has driven G1 around enough to cover the workspace
#
# After this script returns:
#   - /g1_3d_nav_ros2/maps/scans.pcd                   ← new
#   - /g1_3d_nav_ros2/maps/accumulated_grid.pgm        ← new
#   - /g1_3d_nav_ros2/maps/accumulated_grid.yaml       ← new (image path fixed)
#   - /g1_3d_nav_ros2/maps/*.bak                       ← previous map files
#
# To activate: Ctrl-C launch.sh in window A, restart it. open3d_loc
# loads the new PCD; map_server loads the new PGM. Verify ICP fitness
# >= 0.7 with: tail -30 /tmp/loc.log | grep fitness | tail -5
#
# Usage:
#   docker exec -it 3d_nav_ros2 /tmp/mapping_save.sh

set -e

source /opt/ros/humble/setup.bash
source /botbrain_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7448"]'

MAPS=/g1_3d_nav_ros2/maps

echo "[1/5] dumping 3D PCD via fast_lio /map_save..."
ros2 service call /map_save std_srvs/srv/Trigger
# fast_lio writes to its cwd's ./test.pcd (the cwd is /root for launch.sh
# fork; locate by find rather than hard-coding so this still works if
# launch.sh changes its cwd later).
PCD=$(find /root -maxdepth 2 -name 'test.pcd' -newer "$MAPS/scans.pcd" 2>/dev/null | head -1)
if [ -z "$PCD" ]; then
    PCD=$(find /root -maxdepth 2 -name 'test.pcd' 2>/dev/null | head -1)
fi
if [ -z "$PCD" ] || [ ! -s "$PCD" ]; then
    echo "  FAIL: fast_lio did not produce a non-empty test.pcd." >&2
    echo "  Make sure mid360.yaml has 'pcd_save_en: true' and that fast_lio" >&2
    echo "  has been running long enough to accumulate points." >&2
    exit 1
fi
echo "  found new PCD at $PCD"

echo "[2/5] dumping 2D PGM via map_saver_cli..."
# map_saver_cli writes <name>.pgm + <name>.yaml relative to its cwd unless
# the -f path is absolute. We pass an absolute path under /tmp so we don't
# clobber the existing maps until step 4.
ros2 run nav2_map_server map_saver_cli -t /accumulated_grid -f /tmp/_new_grid

echo "[3/5] backing up previous maps to .bak..."
for f in scans.pcd accumulated_grid.pgm accumulated_grid.yaml; do
    if [ -f "$MAPS/$f" ]; then
        cp -f "$MAPS/$f" "$MAPS/${f}.bak"
    fi
done

echo "[4/5] moving new maps into $MAPS/ ..."
mv -f "$PCD" "$MAPS/scans.pcd"
mv -f /tmp/_new_grid.pgm  "$MAPS/accumulated_grid.pgm"
mv -f /tmp/_new_grid.yaml "$MAPS/accumulated_grid.yaml"

echo "[5/5] fixing yaml image path..."
sed -i "s|^image:.*|image: $MAPS/accumulated_grid.pgm|" "$MAPS/accumulated_grid.yaml"

echo
echo "DONE. New map files in $MAPS/ :"
ls -la "$MAPS/scans.pcd" "$MAPS/accumulated_grid.pgm" "$MAPS/accumulated_grid.yaml" 2>&1 | sed 's|^| |'
echo
echo "Next:"
echo "  1. Ctrl+C window A's launch.sh"
echo "  2. Re-run /root/launch.sh — open3d_loc loads new PCD on startup"
echo "  3. Verify ICP fitness >= 0.7:"
echo "       tail -30 /tmp/loc.log | grep fitness | tail -5"
