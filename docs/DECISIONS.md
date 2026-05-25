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
