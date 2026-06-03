#!/bin/bash
# recreate_3d_nav_rgb.sh — destroy + recreate the 3d_nav_rgb runtime container.
# Run on G1 host.

set -e

REPO_DIR="${REPO_DIR:-/home/unitree/g1_3d_nav_rgb}"

if docker ps -a --format '{{.Names}}' | grep -q '^3d_nav_rgb$'; then
    echo "Stopping and removing existing 3d_nav_rgb container..."
    docker stop -t 5 3d_nav_rgb 2>/dev/null || true
    docker rm -f 3d_nav_rgb
fi

docker run -d --name 3d_nav_rgb \
    --network host --ipc host \
    -v "${REPO_DIR}":/g1_3d_nav_rgb \
    -v "${REPO_DIR}/3d_nav_g1/g1_ws/src":/root/3d_nav_g1/g1_ws/src \
    -v /usr/local/lib:/opt/robot_sdk/lib \
    -v /usr/local/include:/opt/robot_sdk/include \
    g1_nav_final:latest sleep infinity

echo ""
echo "=== Container created ==="
docker ps --format 'table {{.Names}}\t{{.Status}}' | grep -E 'NAMES|3d_nav_rgb'
echo ""
echo "Mounts (verify):"
docker inspect 3d_nav_rgb --format '{{range .Mounts}}  {{.Source}} -> {{.Destination}}{{println}}{{end}}'
echo ""
echo "Next: bash /g1_3d_nav_rgb/tools/mapping/mapping_launch_rgb.sh  (inside container)"
