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
`DeclareLaunchArgument('map_file', default_value='/g1_3d_nav_ros2/maps/scans.pcd')`. The
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
- A new mount was needed: `g1_3d_nav_ros2/configs/nav2_params.yaml` is bound
  as a **single-file** read-only mount over
  `/botbrain_ws/install/g1_pkg/share/g1_pkg/config/nav2_params.yaml`. This
  keeps the override outside botbrain's writable workspace and applies the
  moment the host file changes (no docker cp dance).
- `g1_write_node` (and the SDK init crash that comes with it) is not part of
  this PoC. The decision to defer it stands.

## D-009: Nav2 topic overrides for the merged container

**Status:** accepted (2026-05-25), mount mechanism superseded by in-src
fork (2026-05-27 — see end of section).
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

**Decision:** Fork the yaml at `configs/nav2_params.yaml`. Override:
- `static_layer.map_topic: /map_2d`
- `obstacle_layer.cloud.topic: /cloud_registered_body_1` (both costmaps)

Mount it as a single-file read-only bind into the container so botbrain's
launch picks it up at the original install path. Botbrain's source workspace
stays untouched.

**Consequences:** When upstream botbrain bumps its `nav2_params.yaml`, we
manually re-fork. The diff to maintain is small (two stanzas), and the
header comment in the yaml documents the fork rationale for future
maintainers.

**Supersession (2026-05-27):** When botbrain switched to in-repo vendor
+ container-side `colcon symlink-install` (botbrain mount source moved
from host `~/botbrain_ws/` to `<repo>/botbrain/`), the single-file bind
mount over the install path collided with `colcon symlink-install`'s
attempt to symlink that same path back to src ("Device or resource busy").

The fix folded the D-009 fork directly into
`botbrain/src/g1_pkg/config/nav2_params.yaml`:
- 4 × `robot_base_frame: <prefix>base_footprint` → `body`
- `static_layer.map_topic: /map_2d` (added)
- `obstacle_layer.cloud.topic: pointcloud` → `/cloud_registered_body_1`
  (both costmaps)

Runtime behavior is unchanged; the source of truth moved from a separate
fork file (`configs/nav2_params.yaml`) plus a docker single-file mount
to inline edits in the vendored src. The mount is removed from
`tools/recreate_3d_nav_ros2.sh`. Upstream sync now means: pull botbrain
upstream into `botbrain/src/`, re-apply the four edits in-place, commit.

## D-010: Software-only PoC for Nav2 — `g1_write_node` deliberately disabled

**Status:** superseded by D-011 (2026-05-26)
**Context:** The full chain produces `/cmd_vel_out` from twist_mux (NOT /cmd_vel — see test report 2026-05-25); in production
`g1_write_node` consumes it and calls `LocoClient::SetVelocity` on the
Unitree SDK. That node was thought to crash during `ChannelFactory::Init(0)`
with `free(): invalid pointer` — see `Known Issues` in README.

**Decision:** For the merged-container Nav2 PoC, `nav2_launch.sh` does NOT
start `g1_write_node`. The verification chain ends at `/cmd_vel_out` being
populated; the operator confirms with `ros2 topic echo /cmd_vel_out` and RViz2
path rendering, not by walking. G1 stays still.

**Consequences:** The SDK init crash isn't on the critical path for this
milestone. When it's fixed, add a `[3/3] g1_write_node` step to
`nav2_launch.sh` (and possibly a safety brake / e-stop wrapper around it).

**2026-05-26 supersession note:** The "SDK init crash" turned out to be a
diagnostic artifact (a `timeout 8` wrapper sending `SIGTERM` mid-init). On
clean runs `g1_write_node` reaches `active [3]` and a `vx=0.5` Twist on
`/cmd_vel_out` makes G1 step. D-011 takes over — motion is enabled by
default and botbrain's `nav2_params.yaml` is treated as upstream-authoritative.

## D-011: `g1_write_node` is part of `nav2_launch.sh`; botbrain default `nav2_params.yaml` is authoritative

**Status:** accepted (2026-05-26, this is v3 — see "history" at the bottom
for the trajectory of revisions on the same day)
**Context:** R-003 closure (Roadmap, 2026-05-26) verified that
`g1_write_node` reaches `active [3]` cleanly and that publishing
`Twist{linear.x = 0.5}` to `/cmd_vel_out` produces visible motion.
However, that test was a single-Twist `ros2 topic pub --once` ping, which
characterises the **single-step SDK Move dead-zone** — what
velocity must one Move() call request before the SDK triggers a step.
That is **not** the same number as the closed-loop dead-zone Nav2
operates against, where `controller_server` publishes at 20 Hz and the
SDK can integrate the stream into a continuous gait.

The single-step result (`vx=0.05` no motion, `vx=0.5` motion) is a fact
about the SDK API surface. It is not a fact about whether
botbrain's default `nav2_params.yaml` (MPPI, `vx_max=0.35`,
`min_x_velocity_threshold=0.001`) drives this G1 — that is determined by
the closed-loop behaviour, which we have not yet measured.

**Safety review.** A previous draft of this ADR introduced an
`ENABLE_MOTION=1` opt-in flag in `nav2_launch.sh` to gate motion behind
operator consent. Reading `g1_write.cpp` and `bot_bringup/twist_mux.yaml`
showed botbrain already provides multiple safety layers:
- L1 (SDK): `LocoClient::SetTimeout(2.0f)`; G1 FSM rejects Move when not
  in sport mode.
- L2 (mux): twist_mux input timeouts (0.1–0.5 s) drop stale sources;
  `dead_man_switch` lock is wired but unpublished (Roadmap R-005 — the
  one missing piece).
- L3 (node): `/emergency_stop` service in `g1_write` toggles
  `emergency_flag_`, which causes `cmd_vel_subscription_callback` to drop
  Twists and triggers `stop_move()` + squat. **Already available.**
- L3 (node): `/mode` service for explicit FSM control.
- L5 (hw): RC controller L2+B is always available, independent of ROS.

For test and operator-supervised use, L3 + L5 are sufficient. The
`ENABLE_MOTION` flag adds maintenance burden without filling a real gap.
R-005's `dead_man_switch` (fail-deadly) is the only missing layer, and it
matters for **production deploy** (operator may be remote or absent), not
for supervised testing.

<!-- D-011 continues -->
**Considered alternatives:**
1. **Override botbrain's `nav2_params.yaml`** to raise the controller's
   effective output floor above the single-step dead-zone (e.g.
   `min_x_velocity_threshold: 0.5` and `FollowPath.vx_max: 0.7`).
   Rejected — conflates the single-step dead-zone with the closed-loop
   regime, and pre-emptively patches around a gap we have not actually
   observed. Botbrain's defaults were validated upstream on G1; if they
   fail in our stack, the gap is more likely elsewhere (TF, lifecycle,
   topic remaps, controller frequency, costmap, missing publishers) than
   in the velocity numbers themselves.
2. **Velocity dead-zone compensation in `g1_write` callback** — clamp
   non-zero `|vx| < dead_zone` up to `dead_zone`. Same conflation as (1)
   plus breaks SDK semantics; rejected.
3. **Insert `nav2_velocity_smoother`** between `controller_server` and
   `twist_mux` to scale Twists. Same conflation; adds a lifecycle node;
   rejected.
4. **Use botbrain defaults verbatim and let closed-loop behaviour speak
   for itself.** Run e2e walk with the upstream config. If G1 doesn't
   move, diagnose the gap in *our* stack — not the parameter values.

**Decision:** Take option **(4)**. Do not modify `configs/nav2_params.yaml`
beyond what D-009 already covers (`static_layer.map_topic` and
`obstacle_layer.cloud.topic` topic forks — those are real topology
mismatches, not value tuning). `nav2_launch.sh` ships `g1_write_node` as
`[3/3]`, default-on. R-001 fix (auto-activate `zero_vel_publisher`) is
folded in as `[2.5/3]`. No `ENABLE_MOTION` flag. Operator safety
preconditions are documented in `README.md`'s Quick start.

**Consequences:**
- The fork on `nav2_params.yaml` (D-009) stays exactly two stanzas. No
  velocity-tuning entries.
- `nav2_launch.sh` becomes the single all-in-one operator command — the
  previous "PoC mode that doesn't move" is recoverable by skipping the
  `[3/3]` step manually.
- The `ENABLE_MOTION` flag mentioned in earlier drafts of this ADR is
  **not implemented** and is not part of the design.
- If e2e walk fails with botbrain defaults, the failure mode is
  diagnostic input — the gap is in our stack vs. the upstream-validated
  topology; the cure is to find and fix that gap, not to tune the
  parameters.

**Roadmap relationship:**
- R-007 (single-step dead-zone sweep) is reframed as a *characterisation
  task* — interesting for understanding the SDK API but not a Nav2
  blocker.
- R-005 (dead-man-switch) is unchanged: production-deploy blocker, not a
  test blocker.
- R-008 (destructor cyclone race) unchanged: shutdown anomaly, not a
  motion-enablement blocker.

**History:**
- v1 (initial draft, 2026-05-26): "ENABLE_MOTION=1 flag, motion off by
  default" — based on assumed-narrow safety surface.
- v2 (intra-day, after safety review): dropped the flag, kept a 0.5
  velocity-floor override on `nav2_params.yaml`.
- v3 (this version, after closed-loop vs single-step distinction): dropped
  the velocity-floor override too. Botbrain defaults verbatim.

**Verified (2026-05-26 evening):** End-to-end goal-driven walk passes on
G1 with this configuration. RViz2 `2D Goal Pose` → `bt_navigator: Goal
succeeded` inside `xy_goal_tolerance: 0.10`. The closed-loop dead-zone
prediction held — botbrain default `vx_max: 0.35` /
`min_x_velocity_threshold: 0.001` drive G1 fine, despite the single-step
SDK Move dead-zone observed in R-003 closure being above 0.05. See
`docs/TEST_REPORTS/2026-05-26-walk-e2e.md`. The only D-011-relevant
patch beyond D-009's two topic forks turned out to be a frame-name
mismatch (`base_footprint` → `body`) — same character as the topic
forks (real topology mismatch with this G1 stack, not value tuning).
One new caveat split off as Roadmap R-009: the bind-mount overlay of
the fork yaml does not always reach long-lived nav2 processes across
`nav2_launch.sh` restarts; `docker stop && start` is the working
workaround.

