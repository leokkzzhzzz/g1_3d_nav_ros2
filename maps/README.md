# Maps

This directory **is** the runtime mount source. The `3d_nav_ros2`
container mounts it as `/g1_3d_nav_ros2/maps/`, so anything written
inside the container at that path lands here on the host immediately.

## What lives here

| File | Tracked in git | Why |
|---|---|---|
| `accumulated_grid.pgm` | ✅ yes | small (~1.2 MB), rarely changes between mapping runs |
| `accumulated_grid.yaml` | ✅ yes | metadata for the pgm above |
| `scans.pcd` | ❌ no (`.gitignore`) | per-environment; ~258 MB |
| `*.bak` siblings | ❌ no | one-step rollback after `mapping_save.sh` |

## Mount setup

```bash
docker run -d --name 3d_nav_ros2 \
    --network host --ipc host \
    -v /home/unitree/g1_3d_nav_ros2_repo/maps:/g1_3d_nav_ros2/maps \
    g1_nav_final:latest sleep infinity
```

The full container creation command is in
`tools/recreate_3d_nav_ros2.sh`.

## How to obtain `scans.pcd`

### (A) ROS2-native mapping (preferred — same Zenoh fabric)

The ROS2 stack runs `fast_lio` in mapping mode by default. Combined
with `tools/grid_accumulator.py` (a ROS2 port of deepglint's
ground_cloud_accumulator), both the 3D PCD and the 2D PGM can be
produced from inside the `3d_nav_ros2` container:

```bash
# window A — localization stack (publishes /cloud_registered_body_1
# and the map->body TF that grid_accumulator reads)
docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/launch.sh

# window B — 2D grid accumulator
docker exec -it 3d_nav_ros2 /tmp/mapping_record.sh

# RC-drive G1 around the workspace (5–15 minutes), covering each
# future waypoint area from multiple viewpoints

# window C — dump both maps to /g1_3d_nav_ros2/maps/ (this dir)
docker exec -it 3d_nav_ros2 /tmp/mapping_save.sh
```

`mapping_save.sh` writes `scans.pcd`, `accumulated_grid.pgm`, and
`accumulated_grid.yaml` directly into the mount target, so they
appear here on the host with no additional `cp`. Previous map files
are kept at `*.bak` for rollback.

To activate the new maps: Ctrl-C window A's `launch.sh`, restart
it, then `tail -30 /tmp/loc.log | grep fitness | tail -5` should
show ICP fitness ≥ 0.7.

### (B) ROS1 mapping (legacy — pre-D-001)

Before the runtime moved to a single ROS2-rmw_zenoh_cpp deployment,
mapping was done in the HongTu / FAST-LIO ROS1 container. That path
still works:

```bash
roslaunch fast_lio mapping_g1_full.launch rviz:=false
# Drive G1 around to scan the area; Ctrl-C to save
# Output: /root/deepglint_loc/FAST_LIO/PCD/scans.pcd
# Copy to /home/unitree/g1_3d_nav_ros2_repo/maps/scans.pcd
```

See `leokkzzhzzz/g1_3d_nav` (ros1 branch) for the full ROS1 mapping
setup. We keep this as a fallback while the ROS2 mapping pipeline
matures.

## Versioning the PGM after a new mapping run

`mapping_save.sh` writes the new PGM directly into this directory
(this *is* the git working tree on the G1 host). To capture the
update in canon:

```bash
# Leo side
ssh unitree@192.168.100.30 'cat /home/unitree/g1_3d_nav_ros2_repo/maps/accumulated_grid.pgm'  > /tmp/accumulated_grid.pgm
ssh unitree@192.168.100.30 'cat /home/unitree/g1_3d_nav_ros2_repo/maps/accumulated_grid.yaml' > /tmp/accumulated_grid.yaml

cd <canon clone>
cp /tmp/accumulated_grid.{pgm,yaml} maps/
git add maps/accumulated_grid.* && git commit && git push
```

`scans.pcd` stays out of git regardless of how it was generated.
