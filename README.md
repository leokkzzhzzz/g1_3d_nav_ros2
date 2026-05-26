# g1_3d_nav_ros2

ROS 2 Humble native 3D localization runtime for the Unitree G1 Edu humanoid
robot. Replaces the ROS 1 + ros1_bridge path with a single-RMW
(`rmw_zenoh_cpp`) deployment.

> Status: 2026-05-25 — verified end-to-end on G1 (192.168.100.30) with
> cross-host RViz2 from Leo (192.168.100.13). Image:
> `g1_nav_final:latest` SHA `183e0426c630...`.

## What this repo holds

- **`3d_nav_g1/g1_ws/src/`** — full deepglint source tree (FAST_LIO, open3d_loc)
  with all 2026-05-25 patches applied
- **`3d_nav_g1/livox_ws/src/`** — Livox MID360 ROS 2 driver source
- **`3d_nav_g1/deps/open3d141/`** — Open3D 0.14.1 headers + CMake config
  (binary libs separate, see `3d_nav_g1/deps/open3d141/README.md`)
- **`launch.sh`** — 6-step runtime entry script
- **`maps/`** — 2D occupancy grid (3D PCD is build product, not in git — see
  `maps/README.md`)
- **`configs/`** — Leo-side RViz2 config
- **`config/`**, **`patches/`**, **`docs/`** — extra documentation set
  (consolidated configs, C++ patch notes, design decisions, glossary)

The full set of patches in this repo is what makes the stack work — see
`docs/DECISIONS.md` for the 7 architecturally significant decisions.

## Runtime topology

```
G1 (192.168.100.30)                         Leo (192.168.100.13)
┌─ 3d_nav_ros2 container ──────────────┐    ┌─ host install ─┐
│ image: g1_nav_final:latest           │    │ rmw_zenoh_cpp  │
│ net=host, ipc=host                   │    │ rviz2          │
│ /root/maps -> /home/unitree/.../maps │    └────────────────┘
│                                      │            │
│ [1/6] rmw_zenohd          :7448 ◄────┼──tcp/7448──┘ (RMW=rmw_zenoh_cpp client)
│ [2/6] livox_ros_driver2              │
│ [3/6] fast_lio (FAST-LIO odometry)   │
│ [4/6] open3d_loc (ICP global loc)    │
│ [5/6] map_server (/map_2d)           │
│ [6/6] pointcloud_to_laserscan (/scan)│
└──────────────────────────────────────┘
```

All nodes share `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and connect to the
in-container Zenoh router on `tcp/127.0.0.1:7448`. Leo connects to the same
router across the network. No DDS bridge, no ros1_bridge.

## Quickstart

### Prerequisites (one-off)

**On G1 (`192.168.100.30`):**

1. Pull the Docker image:
   ```bash
   docker pull us-central1-docker.pkg.dev/dreamcontroltrain/g1-nav/3d_nav_g1:latest
   docker tag  us-central1-docker.pkg.dev/dreamcontroltrain/g1-nav/3d_nav_g1:latest g1_nav_final:latest
   ```
2. Place `scans.pcd` at `/home/unitree/g1_3d_nav/maps/scans.pcd` (see
   `maps/README.md` for how to obtain).
3. Create the runtime container if it does not exist:
   ```bash
   docker run -d --name 3d_nav_ros2 \
       --network host --ipc host \
       -v /home/unitree/g1_3d_nav/maps:/root/maps \
       g1_nav_final:latest sleep infinity
   ```

**On Leo (`192.168.100.13`):**

```bash
sudo apt install ros-humble-rmw-zenoh-cpp ros-humble-rviz2
```

### Per-run startup

```bash
# 1. G1: stop ROS 1 path containers (if running)
ssh unitree@192.168.100.30 'docker stop -t 2 g1_loc_ros1 g1_bridge 2>/dev/null'

# 2. G1: start the runtime container (if it was stopped)
ssh unitree@192.168.100.30 'docker start 3d_nav_ros2'

# 3. G1: launch the 6-step stack inside the container
ssh unitree@192.168.100.30 \
  'docker exec -d 3d_nav_ros2 bash -c "cd /root && bash launch.sh > /tmp/launch.log 2>&1"'

# 4. Wait ~90 s, then verify all 6 steps OK:
ssh unitree@192.168.100.30 'docker exec 3d_nav_ros2 tail -15 /tmp/launch.log'

# 5. Leo: launch RViz2 connected to the G1 Zenoh router
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.100.30:7448"]'
export ZENOH_ROUTER_CHECK_ATTEMPTS=10
ros2 daemon stop && ros2 daemon start && sleep 4
rviz2 -d configs/g1_track0_rviz2.rviz
```

### Critical first-launch step — set initial pose

`open3d_loc` ICP needs the robot to start within ~1 m of where the offline
PCD says it is. Without it, `/localization_3d_confidence` stays at 0.0,
`map → odom` is never corrected, and `fast_lio` will eventually drift to
non-physical coordinates.

Two ways:

1. **Manually re-position G1** at a known origin and restart `bash /root/launch.sh`.
2. **Use RViz2 "2D Pose Estimate" tool** — publishes `/initialpose`, open3d_loc
   subscribes and re-seeds.

After `/localization_3d_confidence > 0.7`, the chain stays bounded for
30+ minutes of stationary running.

## Daily operation — Nav2 + motion (D-011)

End-to-end goal-driven motion was verified on 2026-05-26: RViz2 `2D Goal
Pose` → planner → controller → twist_mux → `g1_write_node` → SDK
`LocoClient::Move()` → G1 walks. The full stack starts in two SSH sessions:

```bash
# Window A — localization (holds session, Ctrl+C to stop)
ssh unitree@192.168.100.30
docker exec -it 3d_nav_ros2 /root/launch.sh
# wait for "=== ALL 6 NODES RUNNING ==="

# Window B — Nav2 + twist_mux + g1_write_node (holds session)
ssh unitree@192.168.100.30
docker exec -it 3d_nav_ros2 /root/nav2_launch.sh
# wait for "=== STACK READY: G1 motion ENABLED ==="
```

Then on Leo (RViz2 already attached): **2D Pose Estimate** to seed
`open3d_loc` initial pose, **2D Goal Pose** to send a navigation goal.
G1 walks toward the goal.

### Safety preconditions (operator's responsibility)

`nav2_launch.sh` enables motion by default. The operator must satisfy
all four before sending a goal:

1. Operator on site, can see G1 directly.
2. ≥ 1 m clearance around G1; not on a ledge.
3. RC controller in hand; **L2 + B is the hardware brake** (independent
   of the ROS stack).
4. At least one in-stack brake reachable. Two are provided.

For unattended / production deploy, the dead-man-switch (Roadmap R-005)
is required. For supervised testing, the four preconditions above are
sufficient.

### Two in-stack brakes — pick by intent

| Tool | Behaviour | When to use |
|---|---|---|
| `tools/soft_stop.sh` | Cancels all `/navigate_to_pose` goals → twist_mux fallback to `cmd_vel_zero` (priority 1) → G1 stops **standing in sport mode** | Routine "stop the test". G1 immediately ready to accept a new goal. |
| `tools/estop.sh` | Calls `/emergency_stop` service → `emergency_flag_` set, SDK `stop_move()`, then `BALANCE_SQUAT_SQUAT_STAND` → G1 **stops + squats**. Toggle: second call clears `emergency_flag_` and G1 stands back up. | Real emergency. Fail-passive: even if balance fails mid-stop, G1 lands in a low stable posture. |

Both wrappers are designed to be `docker cp`'d into the container under
`/tmp/`, so the operator-side invocation is single-line:

```bash
docker exec -it 3d_nav_ros2 /tmp/soft_stop.sh   # routine
docker exec -it 3d_nav_ros2 /tmp/estop.sh       # emergency (squats)
```

The wrappers self-source ROS env and `RMW_IMPLEMENTATION=rmw_zenoh_cpp`
so they work regardless of the caller shell.

Quick sync into a freshly-started container (Leo side):

```bash
gh api repos/leokkzzhzzz/g1_3d_nav_ros2/contents/tools/soft_stop.sh --jq .content | base64 -d > /tmp/soft_stop.sh
gh api repos/leokkzzhzzz/g1_3d_nav_ros2/contents/tools/estop.sh     --jq .content | base64 -d > /tmp/estop.sh
scp /tmp/soft_stop.sh /tmp/estop.sh unitree@192.168.100.30:/tmp/
ssh unitree@192.168.100.30 'docker cp /tmp/soft_stop.sh 3d_nav_ros2:/tmp/ && docker cp /tmp/estop.sh 3d_nav_ros2:/tmp/ && docker exec 3d_nav_ros2 chmod +x /tmp/soft_stop.sh /tmp/estop.sh'
```

### Troubleshooting — `base_footprint frame does not exist` after restart

Symptom: `/tmp/nav2.log` floods with
`Invalid frame ID "base_footprint" passed to canTransform`. RViz2 sees
goal acks but no plan polyline; G1 doesn't move.

Root cause: the host-side D-009 fork sets `robot_base_frame: body`, and
this is bind-mounted single-file over
`/botbrain_ws/install/g1_pkg/share/g1_pkg/config/nav2_params.yaml`.
Static checks (`docker inspect`, `cat` inside the container) all show
the mount as expected, but in some `nav2_launch.sh` restarts the running
nav2 processes still see the upstream `base_footprint`. The mount
overlay appears to need a full container re-start to take effect cleanly.

Workaround:
```bash
docker stop 3d_nav_ros2 && docker start 3d_nav_ros2
# then re-run launch.sh + nav2_launch.sh in two windows
```
After this, `grep -c base_footprint /tmp/nav2.log` should be `0` and the
goal-driven motion loop works. Tracked as Roadmap R-009.

## Verification

### Container-internal sanity

```bash
docker exec -it 3d_nav_ros2 bash
source /opt/ros/humble/setup.bash
source /root/3d_nav_g1/g1_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7448"]'

# Topic rates
for t in /Odometry_loc /localization_3d /map_2d /tf /scan; do
    rate=$(timeout 4 ros2 topic hz $t 2>&1 | grep -oP "average rate: \K[0-9.]+" | head -1)
    echo "  $t: ${rate:-NO_DATA} Hz"
done
# Expect: ~10 Hz on each odom/loc/scan; /map_2d is latched
```

### Cross-host (Leo) sanity

```bash
ros2 topic list                                      # should see /map /map_2d /scan /tf /Odometry_loc /localization_3d
ros2 topic echo /scan --once | head -5               # LaserScan, frame_id=body
ros2 run tf2_ros tf2_echo map body                   # metric translation, not km-scale
ros2 topic info -v /map | grep Durability            # should show TRANSIENT_LOCAL
```

## Building from source (only if image is unavailable)

The build process is documented separately because it requires Open3D 0.14.1
binaries that are not in this git repo. See
`3d_nav_g1/deps/open3d141/README.md` for how to obtain Open3D libs.

Once `3d_nav_g1/deps/open3d141/lib/*.a` are present:

```bash
cd 3d_nav_g1/livox_ws && colcon build --symlink-install
cd ../g1_ws && source ../livox_ws/install/setup.bash
colcon build --symlink-install
```

For most users: just use the pre-built `g1_nav_final:latest` image. This repo
documents the configuration, not the build pipeline.

## Configuration reference

### `pointcloud_to_laserscan` parameters in `launch.sh`

These mirror ROS 1's `point_to_scan.launch` one-for-one:

| param                  | value         | rationale                                    |
|------------------------|---------------|----------------------------------------------|
| `target_frame`         | `body`        | reproject to robot body for nav              |
| `transform_tolerance`  | `0.01`        | match ROS 1                                  |
| `min_height`           | `-1.0`        | include floor-level obstacles                |
| `max_height`           | `0.15`        | ignore overhead clutter                      |
| `angle_min` / `max`    | `±π`          | full 360°                                    |
| `angle_increment`      | `0.007`       | ~0.4° resolution                             |
| `range_min` / `max`    | `0.2` / `100` | full LiDAR range                             |
| `use_inf`              | `true`        | mark out-of-range as inf for Nav2            |
| `inf_epsilon`          | `1.0`         | inf substitution                             |

### `fast_lio` `mid360.yaml` — must-have entries

- `common.lid_topic: "/livox/lidar"` — must match livox_ros_driver2's actual
  topic name. `/livox/custom_msg` does NOT work (fast_lio hangs at
  "Node init finished").
- `common.imu_topic: "/livox/imu"`
- `mapping.extrinsic_T: [-0.011, -0.02329, 0.04412]`
- `mapping.extrinsic_R: identity`
- `mapping.extrinsic_est_en: true` — leave true; setting false on this image
  hangs the binary at init (root cause not yet identified).

### `open3d_loc` `loc_param_g1.yaml` — Kalman parameter naming

Use **slash-style** keys (`kf_baselink2map/x: [...]`). Nested YAML
(`kf_baselink2map: { x: [...] }`) is silently ignored by the C++ side because
`declare_parameter` uses the literal slash form.

### `/map` publisher QoS

`global_localization_node` publishes `/map` with `KeepLast(1) +
TRANSIENT_LOCAL + RELIABLE`. Enables late-joining RViz2 to receive the latched
PCD. Default ROS 2 publishers are VOLATILE; ROS 1 latched semantics are NOT
the default. See `patches/README.md`.

## Known issues

| issue                                  | impact                                     | workaround                                       |
|----------------------------------------|--------------------------------------------|--------------------------------------------------|
| `extrinsic_est_en: false` hangs binary | can't use ROS 1's value                    | leave `true`, manually set initial pose          |
| `ros2 cli` intermittent context error  | `topic hz` etc. sometimes errors out       | `ros2 daemon stop && ros2 daemon start`          |
| `<defunct>` zombies in container       | cosmetic only                              | `docker restart 3d_nav_ros2` to reap             |
| RViz2 `tf_static TypeHashNotSupported` | log noise from rmw_zenoh_cpp 0.1.8         | ignore — actual transforms work fine             |

## Versioning

- **2026-05-25** — initial public release. Image: `g1_nav_final:latest`
  SHA `183e0426c630...`. Includes: open3d_loc PCD-path fix + `/scan` remap +
  `/map` QoS = TRANSIENT_LOCAL; launch.sh P3 (RMW=zenoh) refactor;
  pcl2laserscan ROS 1 parity; mid360.yaml `lid_topic` correction.

## License

Configurations and scripts: same as upstream — Apache 2.0 / BSD per individual
file. This repo aggregates configurations and patches against deepglint's
open3d_loc and FAST_LIO; consult those projects for their respective licenses.
