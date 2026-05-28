#!/bin/bash
# tools/mapping_rviz2.sh — Launch RViz2 on the operator workstation,
# connected to the G1 Zenoh router for the mapping view. Use while
# tools/mapping/mapping_launch.sh is running on G1.
#
# Pre-conditions:
#   - G1 has tools/mapping/mapping_launch.sh running (4-node mapping stack)
#   - workstation has ros-humble-rmw-zenoh-cpp + ros-humble-rviz2
#
# Usage (from the cloned repo root or any cwd):
#   bash tools/mapping_rviz2.sh
set -e

REPO="$(cd "$(dirname "$0")/.." && pwd)"
RVIZ_CFG="$REPO/configs/g1_mapping_rviz2.rviz"

if [ ! -f "$RVIZ_CFG" ]; then
    echo "ERROR: RViz config not found: $RVIZ_CFG" >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.100.30:7448"]'
export ZENOH_ROUTER_CHECK_ATTEMPTS=10

ros2 daemon stop  >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true
sleep 4

exec rviz2 -d "$RVIZ_CFG"
