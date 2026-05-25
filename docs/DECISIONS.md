# Decision log — `3d_nav_ros2`

Architecturally significant decisions that shaped this stack, in chronological
order.

## D-001: P3 path — single RMW (rmw_zenoh_cpp) end-to-end

**Status:** accepted (2026-05-25)
**Context:** Original ROS 1 → ros1_bridge → DDS → zenoh_bridge_dds → Leo path
hit type-hash incompatibilities that blocked Layer 3 of the proof-of-concept.
Leo end could not subscribe to `/tf`, `/map`, `/scan` reliably across the bridge.

**Decision:** Drop the ros1_bridge path entirely. Run all 5 runtime nodes plus
`rmw_zenohd` under `RMW_IMPLEMENTATION=rmw_zenoh_cpp`. Leo end uses the same
RMW and connects directly to G1's `tcp/0.0.0.0:7448` Zenoh router.

**Consequences:**
- Cross-host topic discovery works without bridge configuration.
- Type-hash compatibility is automatic since both ends use the same RMW.
- ROS 1 fallback containers (`g1_loc_ros1`, `g1_bridge`) are kept stopped, not
  removed, so emergency rollback is one `docker start` away.
- rmw_zenoh_cpp 0.1.8 logs `tf_static TypeHashNotSupported` errors that look
  scary but are cosmetic; ignored in our verification matrix.

## D-002: open3d_loc PCD path is parameterised

**Status:** accepted (2026-05-25)
**Context:** Upstream `open3d_loc_g1.launch.py` hardcoded
`/home/sax/GO2_Localization_ROS2/.../1.test.ply`, an absolute path on a
contributor's dev machine. On G1 this path doesn't exist, the launch fails
with `RPly: Unable to open file`, and the C++ node aborts on the next
`create_publisher()` call due to invalid rcl context.

**Decision:** Replace the hardcoded path with
`DeclareLaunchArgument('map_file', default_value='/root/maps/scans.pcd')`. The
default points at the bind-mounted host directory so swapping maps is a
container-restart-free operation.

**Consequences:** Multi-environment use without source edits.

## D-003: Kalman parameters must use slash-style YAML keys

**Status:** accepted (2026-05-25)
**Context:** The C++ `global_localization_node` calls
`declare_parameter<std::vector<double>>("kf_baselink2map/x")`. ROS 2's nested
YAML rewriting converts `{ kf_baselink2map: { x: [0.001, 0.002] } }` into the
parameter name `kf_baselink2map.x`. Names mismatch, so the C++ side reads
default-constructed empty vectors. Symptom: Kalman filter is mathematically
inert, fitness stays around 0.5, never crosses the 0.7 publish threshold.

**Decision:** Author Kalman params as `kf_baselink2map/x: [...]` directly at the
top level under `ros__parameters`.

**Consequences:** YAML syntactically uglier but functionally correct.

## D-004: `/map` (and other latched topics) need transient_local QoS

**Status:** accepted (2026-05-25)
**Context:** open3d_loc publishes `/map` (the scans.pcd PCD) once at startup.
ROS 2's default `create_publisher<T>("/map", 1)` is `KeepLast(1)+VOLATILE`,
meaning late-joining subscribers receive nothing. RViz2 is always a late
joiner — opening RViz2 after launch.sh starts shows
`Showing [0] points from [0] messages`.

**Decision:** Patch the publisher to
`rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable()`. This restores
ROS 1 latched semantics: the most recent message is buffered for late
subscribers.

**Consequences:** A small upstream patch held in `patches/`. If `/submap`,
`/scan2map`, `/baselink2map`, etc. are later needed for late-joining RViz2
sessions, apply the same transformation.

## D-005: `/scan` topic name remap for open3d_loc internal publisher

**Status:** accepted (2026-05-25)
**Context:** `global_localization_node` itself publishes a debug `PointCloud2`
on `/scan` (intended for the developer's own RViz). `pointcloud_to_laserscan`
also publishes a `LaserScan` on `/scan`. FastDDS tolerates same-name
different-type topics; rmw_zenoh_cpp does not. `ros2 topic echo /scan` reports
"more than one type" and Nav2 cannot subscribe.

**Decision:** Remap the open3d_loc internal publisher to `/scan_loc` via the
launch file's `remappings=[('/scan', '/scan_loc')]` argument. The standard
`/scan` namespace is reserved for the LaserScan that nav2 consumes.

**Consequences:** No source edit required. RViz2 configurations that previously
showed the open3d_loc PointCloud2 on `/scan` need to switch to `/scan_loc`.

## D-006: `pointcloud_to_laserscan` parameter parity with ROS 1

**Status:** accepted (2026-05-25)
**Context:** The ROS 2 launch.sh shipped with conservative defaults
(front-180° / 0.15-0.25 m height band / 20 m range), which made Nav2 blind to
half the LiDAR cloud and to anything below knee height. ROS 1's deployment used
full 360° / -1 m to 0.15 m / 100 m range.

**Decision:** Mirror ROS 1's `point_to_scan.launch` parameter set directly.
Documented as a table in the README.

**Consequences:** Larger LaserScan, more compute (linear in point count, still
under MID360's 10 Hz budget). Lower height bound captures floor-level obstacles
that the previous band was missing.

## D-007: Initial pose is an operator responsibility, not auto-recovery

**Status:** accepted (2026-05-25)
**Context:** When G1 is placed at a location far from the PCD's mapping origin,
ICP fitness stays low, `map → odom` is never corrected, and pure-IMU integration
in fast_lio runs unbounded — eventually the position grows by hundreds of
kilometres and the run is unrecoverable without a fresh launch.sh.

**Considered alternatives:** A watchdog node that monitors `/Odometry_loc`
magnitude and force-restarts fast_lio when it exceeds a threshold; making
`extrinsic_est_en: false` to disable IMU bias drift; expanding ICP search
radius.

**Decision:** Document the requirement in the runbook (set initial pose either
by physically positioning G1 at a known origin or via RViz2 "2D Pose Estimate")
and don't auto-recover. Auto-restarting fast_lio mid-run loses odometry
continuity for any consumer that relies on monotonic poses.

**Consequences:** First-launch operator step; no software fallback. If this
turns out to be too operationally fragile, revisit and add a watchdog.


## D-008: Merged Nav2 into the 3d_nav_ros2 container

**Status:** accepted (2026-05-25)
**Context:** The earlier two-container topology (`3d_nav_ros2` for localization,
`g1_nav_bb` for Nav2 + control) worked but doubled the operational surface.
Both containers had to be RMW-aligned to `rmw_zenoh_cpp`, both had to point at
the same Zenoh router, and any topic / TF mismatch had to be debugged across
two writable layers.

**Considered alternatives:**
- Keep two containers (status quo) — simpler image surgery, more startup steps.
- Merge Nav2 into `3d_nav_ros2` — one container, one writable layer.
- Inverse merge (localization into `g1_nav_bb`'s `dustynv` image) — that image
  ships without deepglint Open3D / FAST_LIO; rebuilding would take hours.

**Decision:** Merge. `apt install` the Nav2 stack and `twist_mux` into
`g1_nav_final:latest`, mount the existing `botbrain_ws` plus the robot SDK
header/library directories from the host, and run both stacks as in-process
peers under the same Zenoh router on `127.0.0.1:7448`.

**Consequences:**
- Image grew by ~250 MB (acceptable; one-off cost).
- A new mount was needed: `g1_3d_nav_ros2/config/nav2_params.yaml` is bound
  as a **single-file** read-only mount over
  `/botbrain_ws/install/g1_pkg/share/g1_pkg/config/nav2_params.yaml`. This
  keeps the override outside botbrain's writable workspace and applies the
  moment the host file changes (no docker cp dance).
- `g1_write_node` (and the SDK init crash that comes with it) is not part of
  this PoC. The decision to defer it stands.

## D-009: Nav2 topic overrides for the merged container

**Status:** accepted (2026-05-25)
**Context:** Botbrain's `nav2_params.yaml` was authored against a robot whose
sensor topology differs from this stack:
- `static_layer` defaults to subscribing `/map`, but in our stack `/map`
  is the open3d_loc PointCloud2 PCD (latched, transient_local). The
  OccupancyGrid lives on `/map_2d`.
- `obstacle_layer.cloud.topic: pointcloud` doesn't exist; FAST-LIO publishes
  the body-frame point cloud on `/cloud_registered_body_1`.

**Considered alternatives:**
- Run-time `--ros-args -r` remaps in `nav2_launch.sh` (no yaml edit).
- Modify botbrain's source `nav2_params.yaml` and re-build the workspace
  (touches upstream code).
- Fork the yaml inside our repo (this project's choice).

**Decision:** Fork the yaml at `config/nav2_params.yaml`. Override:
- `static_layer.map_topic: /map_2d`
- `obstacle_layer.cloud.topic: /cloud_registered_body_1` (both costmaps)

Mount it as a single-file read-only bind into the container so botbrain's
launch picks it up at the original install path. Botbrain's source workspace
stays untouched.

**Consequences:** When upstream botbrain bumps its `nav2_params.yaml`, we
manually re-fork. The diff to maintain is small (two stanzas), and the
header comment in the yaml documents the fork rationale for future
maintainers.

## D-010: Software-only PoC for Nav2 — `g1_write_node` deliberately disabled

**Status:** accepted (2026-05-25)
**Context:** The full chain produces `/cmd_vel` from twist_mux; in production
`g1_write_node` consumes it and calls `LocoClient::SetVelocity` on the
Unitree SDK. That node currently crashes during `ChannelFactory::Init(0)`
with `free(): invalid pointer` — see `Known Issues` in README.

**Decision:** For the merged-container Nav2 PoC, `nav2_launch.sh` does NOT
start `g1_write_node`. The verification chain ends at `/cmd_vel` being
populated; the operator confirms with `ros2 topic echo /cmd_vel` and RViz2
path rendering, not by walking. G1 stays still.

**Consequences:** The SDK init crash isn't on the critical path for this
milestone. When it's fixed, add a `[3/3] g1_write_node` step to
`nav2_launch.sh` (and possibly a safety brake / e-stop wrapper around it).
