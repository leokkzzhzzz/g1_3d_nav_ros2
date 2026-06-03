#!/bin/bash
# mapping_save_rgb.sh — collect the FAST-LIVO2 colored PCD and install it
# as /g1_3d_nav_rgb/maps/scans.pcd.
#
# FAST-LIVO2 writes Log/PCD/all_raw_points.pcd (colored, binary PCL) on
# SIGINT/SIGTERM via its signalHandler. This script waits for that file,
# validates it, and copies it to the canonical map path.
#
# Called automatically by mapping_launch_rgb.sh on Ctrl+C, or manually.
#
# Exit codes:
#   0  — scans.pcd written and validated
#   1  — source PCD missing or too small
#   2  — copy failed
#
# Usage (inside 3d_nav_rgb container):
#   bash /g1_3d_nav_rgb/tools/mapping/mapping_save_rgb.sh

MAPS=/g1_3d_nav_rgb/maps
DEST="$MAPS/scans.pcd"

# FAST-LIVO2 ROOT_DIR is the install share dir at runtime, but savePCD()
# writes relative to the compiled ROOT_DIR (source tree inside container).
# The container mounts src at /root/3d_nav_g1/g1_ws/src, so:
LIVO2_SRC=/root/3d_nav_g1/g1_ws/src/deepglint/FAST_LIVO2
SRC_PCD="$LIVO2_SRC/Log/PCD/all_raw_points.pcd"

MIN_PCD_BYTES=$((1 * 1024 * 1024))   # 1 MB floor
T0=$(date +%s)

mkdir -p "$MAPS"
mkdir -p "$LIVO2_SRC/Log/PCD"

echo "[1/2] waiting for FAST-LIVO2 to write $SRC_PCD ..."
for i in $(seq 1 30); do
    if [ -f "$SRC_PCD" ]; then
        MTIME=$(stat -c %Y "$SRC_PCD" 2>/dev/null || echo 0)
        SIZE=$(stat -c %s  "$SRC_PCD" 2>/dev/null || echo 0)
        if [ "$MTIME" -ge "$T0" ] && [ "$SIZE" -ge "$MIN_PCD_BYTES" ]; then
            echo "  found: $SRC_PCD ($SIZE bytes)"
            break
        fi
    fi
    sleep 1
done

if [ ! -f "$SRC_PCD" ]; then
    echo "  FAIL: $SRC_PCD not found after 30s. Was pcd_save_en: true in mid_360.yaml?" >&2
    exit 1
fi
SIZE=$(stat -c %s "$SRC_PCD" 2>/dev/null || echo 0)
MTIME=$(stat -c %Y "$SRC_PCD" 2>/dev/null || echo 0)
if [ "$SIZE" -lt "$MIN_PCD_BYTES" ]; then
    echo "  FAIL: $SRC_PCD size $SIZE < $MIN_PCD_BYTES (looks truncated)." >&2
    exit 1
fi
if [ "$MTIME" -lt "$T0" ]; then
    echo "  FAIL: $SRC_PCD mtime older than save start (stale file from previous run)." >&2
    exit 1
fi

echo "[2/2] installing $SRC_PCD -> $DEST ..."
if ! cp "$SRC_PCD" "$DEST"; then
    echo "  FAIL: cp returned non-zero." >&2
    exit 2
fi

echo ""
echo "DONE."
ls -lh "$DEST"
echo ""
echo "Validate colors:"
echo "  python3 -c \""
echo "  import open3d as o3d"
echo "  pcd = o3d.io.read_point_cloud('$DEST')"
echo "  print('Has colors:', pcd.has_colors())"
echo "  print('Points:', len(pcd.points))"
echo "  \""
exit 0
