#!/bin/bash
# recreate_3d_nav_ros2.sh — destroy + recreate the runtime container with
# all required mounts for the merged-container Nav2 deployment.
#
# Run on G1 host. Uses the latest committed g1_nav_final:latest image (must
# include nav2 + twist_mux apt packages — see tools/install_nav2.sh).

set -e

REPO_DIR="${REPO_DIR:-/home/unitree/g1_3d_nav_ros2_repo}"

# Stop & remove old container if it exists
if docker ps -a --format '{{.Names}}' | grep -q '^3d_nav_ros2$'; then
    echo "Stopping and removing existing 3d_nav_ros2 container..."
    docker stop -t 5 3d_nav_ros2 2>/dev/null || true
    docker rm -f 3d_nav_ros2
fi

# Recreate with all mounts. The repo working tree is bind-mounted whole
# at /g1_3d_nav_ros2 — tools/, maps/, docs/ all show up as subdirs, and
# `git pull` on the host immediately makes new tools available inside
# the container (no docker cp).
#
# botbrain: bind-mounted at /botbrain_ws so colcon symlink-install lands
# install/build/log on the host working tree (gitignored). The D-009
# fork (robot_base_frame: body, /map_2d, /cloud_registered_body_1) is
# applied directly inside botbrain/src/g1_pkg/config/nav2_params.yaml,
# not via a single-file mount — see DECISIONS D-009 supersession note.
docker run -d --name 3d_nav_ros2 \
    --network host --ipc host \
    -v /home/unitree/g1_3d_nav_ros2_repo:/g1_3d_nav_ros2 \
    -v /home/unitree/g1_3d_nav_ros2_repo/botbrain:/botbrain_ws \
    -v /home/unitree/g1_3d_nav_ros2_repo/3d_nav_g1/g1_ws/src:/root/3d_nav_g1/g1_ws/src \
    -v /usr/local/lib:/opt/robot_sdk/lib \
    -v /usr/local/include:/opt/robot_sdk/include \
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
