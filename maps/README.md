# Maps

The 2D occupancy grid (`accumulated_grid.pgm` + `accumulated_grid.yaml`) is
versioned in this repo because it is small (~1.2 MB) and rarely changes.

The 3D PCD map (`scans.pcd`, ~258 MB) is **not** versioned. It is a build
product of the offline mapping run (HongTu / FAST-LIO mapping container) and
should be re-generated per environment.

## Where to put `scans.pcd` at runtime

Bind-mount the host directory containing `scans.pcd` into the container so
that it appears at `/root/maps/scans.pcd`:

```bash
docker run -d --name 3d_nav_ros2 \
    --network host --ipc host \
    -v /home/unitree/g1_3d_nav/maps:/root/maps \
    g1_nav_final:latest sleep infinity
```

## How to obtain `scans.pcd`

- For the original mapping environment, ask the team for the existing
  `scans.pcd` (~258 MB).
- For a new environment, run the offline mapping pipeline:

```bash
# In the ROS 1 mapping container (HongTu)
roslaunch fast_lio mapping_g1_full.launch rviz:=false
# Drive G1 around to scan the area; Ctrl-C to save
# Output: /root/deepglint_loc/FAST_LIO/PCD/scans.pcd
# Copy it to /home/unitree/g1_3d_nav/maps/scans.pcd
```

See `leokkzzhzzz/g1_3d_nav` (ros1 branch) for the full mapping setup.
