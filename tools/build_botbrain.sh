#!/bin/bash
#
# build_botbrain.sh — colcon build the in-repo botbrain workspace inside
# the running 3d_nav_ros2 container.
#
# Why this exists:
#   The repo vendors botbrain at <repo>/botbrain/ (src only — install,
#   build, log are .gitignored per botbrain/.gitignore). The runtime
#   container mounts <repo>/botbrain → /botbrain_ws (see
#   tools/recreate_3d_nav_ros2.sh), so colcon build inside the container
#   produces install/build/log on the host working tree. New clones of
#   this repo must run this script once to populate install/ before
#   launch.sh / nav2_launch.sh can come up.
#
# Prerequisites:
#   - 3d_nav_ros2 container running (bash tools/recreate_3d_nav_ros2.sh)
#   - g1_nav_final:latest image already has nav2 + twist_mux apt packages
#     (D-008) and the ROS 2 humble toolchain
#
# Usage:
#   bash tools/build_botbrain.sh
#
# Idempotent. Subsequent runs do incremental colcon build.

set -euo pipefail

CONTAINER="${CONTAINER:-3d_nav_ros2}"

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}\$"; then
    echo "ERROR: container '${CONTAINER}' is not running." >&2
    echo "  Run: bash tools/recreate_3d_nav_ros2.sh" >&2
    exit 1
fi

echo "=== colcon build /botbrain_ws inside ${CONTAINER} ==="
echo "(symlink-install — install will be a tree of symlinks back into src)"
echo ""

docker exec "${CONTAINER}" bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    # Unitree SDK2 headers/libs are bind-mounted at /opt/robot_sdk in
    # the container (not /usr/local). g1_pkg/CMakeLists.txt reads the
    # CMake variable UNITREE_SDK2_ROOT (not the env var), so it must
    # be passed via --cmake-args.
    cd /botbrain_ws
    colcon build --symlink-install \
        --cmake-args -DUNITREE_SDK2_ROOT=/opt/robot_sdk
'

echo ""
echo "Done. Verify:"
echo "  ls /home/unitree/g1_3d_nav_ros2_repo/botbrain/install"
echo "Next: bash launch.sh inside the container, then bash nav2_launch.sh."
