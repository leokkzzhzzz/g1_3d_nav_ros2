#!/bin/bash
# recreate_3d_nav_ros2.sh — destroy + recreate the runtime container with
# all required mounts for the merged-container Nav2 deployment.
#
# Run on G1 host. Uses the latest committed g1_nav_final:latest image (must
# include nav2 + twist_mux apt packages — see scripts/install_nav2.sh).

set -e

REPO_DIR="${REPO_DIR:-/home/unitree/g1_3d_nav_ros2_repo}"
NAV2_PARAMS="${REPO_DIR}/config/nav2_params.yaml"

# Sanity: yaml override file must exist on host
if [ ! -f "$NAV2_PARAMS" ]; then
    echo "ERROR: $NAV2_PARAMS missing on host. git pull g1_3d_nav_ros2_repo first." >&2
    exit 1
fi

# Stop & remove old container if it exists
if docker ps -a --format '{{.Names}}' | grep -q '^3d_nav_ros2$'; then
    echo "Stopping and removing existing 3d_nav_ros2 container..."
    docker stop -t 5 3d_nav_ros2 2>/dev/null || true
    docker rm -f 3d_nav_ros2
fi

# Recreate with all mounts
docker run -d --name 3d_nav_ros2 \
    --network host --ipc host \
    -v /home/unitree/g1_3d_nav/maps:/root/maps \
    -v /home/unitree/botbrain_ws:/botbrain_ws \
    -v /usr/local/lib:/opt/robot_sdk/lib \
    -v /usr/local/include:/opt/robot_sdk/include \
    -v "$NAV2_PARAMS":/botbrain_ws/install/g1_pkg/share/g1_pkg/config/nav2_params.yaml:ro \
    g1_nav_final:latest sleep infinity

echo ""
echo "=== Container created ==="
docker ps --format 'table {{.Names}}\t{{.Status}}' | grep -E 'NAMES|3d_nav_ros2'
echo ""
echo "Mounts (verify):"
docker inspect 3d_nav_ros2 --format '{{range .Mounts}}  {{.Source}} -> {{.Destination}}{{println}}{{end}}'
echo ""
echo "Next: bash launch.sh inside the container (defines /tmp/launch.log)"
echo "      then bash nav2_launch.sh (defines /tmp/nav2.log + /tmp/twist_mux.log)"
