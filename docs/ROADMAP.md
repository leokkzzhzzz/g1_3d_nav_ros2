# Roadmap

Active improvement targets for the `g1_3d_nav_ros2` runtime. Items move from
**open** → **in progress** → **done**. Done items are not deleted; they
remain with a date and short note so the history of the stack is readable
from this one file.

Last reviewed: 2026-05-25.

## Open

### R-001 — Auto-activate `zero_vel_publisher` in `nav2_launch.sh`
**Symptom:** After `bash nav2_launch.sh`, `zero_vel_publisher` is in
`unconfigured` state. The `OnProcessStart` event handler in
`bot_bringup/twist_mux.launch.py` does not transition the node under
rmw_zenoh_cpp. As a result, `/cmd_vel_zero` is silent, twist_mux has no
priority-1 fallback, and `/cmd_vel_out` goes idle whenever Nav2 stops
publishing `/cmd_vel_nav`.

**Fix:** Add to `scripts/nav2_launch.sh` after `[2/2] twist_mux ... OK`:
```bash
sleep 2
ros2 lifecycle set /zero_vel_publisher configure
ros2 lifecycle set /zero_vel_publisher activate
```
And `wait_for` the resulting `active [3]` state before declaring the stack
ready.

**Why not fix upstream botbrain:** the launch file works on the platform
botbrain was built for; the regression is environment-specific. Fixing
locally first lets us decide later whether to upstream.

**Effort:** 5 min edit + tracer re-run.

---

### R-002 — Container `ros2 cli` instability under rmw_zenoh_cpp
**Symptom:** `ros2 topic hz`, `ros2 topic echo`, `ros2 topic info -v`
intermittently fail with `failed to initialize wait set: the given context
is not valid` inside the `3d_nav_ros2` container.

**Workaround:** invoke from Leo end (host-side `ros2 cli` is stable on the
same Zenoh router) or restart `ros2 daemon`.

**Real fix candidates:**
- Upgrade to `rmw_zenoh_cpp` 0.2.x once available for Humble (current
  pinned version is 0.1.8).
- File issue against `ros2/rmw_zenoh` with reproduction.

**Effort:** ~1 day investigation, fix is upstream so unbounded.

---

### R-004 — Costmap obstacle-avoidance stress test
**Status:** Not exercised. The single tracer goal in 2026-05-25's report
went through unobstructed terrain.

**Test:** With G1 stationary, place a physical obstacle in the open path,
send a goal that requires a detour. Verify the obstacle layer marks the
costmap and the planner re-plans around it.

**Why this matters:** the merge of `static_layer.map_topic: /map_2d` and
`obstacle_layer.cloud.topic: /cloud_registered_body_1` is documented in
ADR D-009 but only tested in a no-obstacle scenario.

**Effort:** ~30 min once stack is up.

---

### R-005 — `/dead_man_switch` semantics + safety brake
**Status:** twist_mux's `dead_man_switch` lock is configured (`priority:
200`, `timeout: 0.0`) but no node currently publishes to it. With nothing
publishing, the lock is effectively absent — twist_mux passes through
freely. This is the opposite of safe-by-default.

**Decision needed:** Now that R-003 is closed and motion is enabled (ADR
D-011), do we add an explicit publisher for `/dead_man_switch` and require
a Leo-side button or gamepad press to release it? Or accept the current
"no lock without publisher" behavior?

**Scope (2026-05-26 update):** R-005 is a **production-deploy blocker**,
not a test blocker. ADR D-011's safety review concluded that the existing
`/emergency_stop` service in `g1_write` plus the RC controller plus
on-site operator presence is sufficient for supervised testing. R-005
fixes the case where the operator may be remote or absent. Take it on
before any remote/unattended use.

---

### R-006 — Upstream contributions to botbrain
Once R-001 is closed, consider PRs back to botbrain:
- `nav2_params.yaml` topic indirection (parameterise `static_layer.map_topic`
  and `obstacle_layer.cloud.topic` instead of hardcoding) so external
  consumers don't need a fork.
- Whatever turns out to be the reliable lifecycle-activation pattern in
  rmw_zenoh_cpp deployments.

---

### R-007 — Single-step SDK Move dead-zone characterisation
**Status:** Characterisation task, not a Nav2 blocker. ADR D-011 v3
establishes that single-Twist `--once` results (`vx=0.05` no motion,
`vx=0.5` motion) characterise the **single-step** SDK Move dead-zone,
which is *not* the same regime as the closed-loop 20 Hz Twist stream Nav2
drives. Botbrain's default `nav2_params.yaml` is the upstream-validated
config; this measurement does not feed into Nav2 tuning.

**Test:** With `g1_write_node` active, sweep `vx ∈ {0.05, 0.10, 0.15,
0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50}` (and the same for `vy` and
`wz`), each as `ros2 topic pub --once /cmd_vel_out`. Record at which
value G1 first responds.

**Why this still matters:** answers operational questions like "if a
future custom controller drops below the closed-loop frequency, how slow
can it go before the SDK ignores it" and "does the dead-zone differ
across axes". Useful operational knowledge, not a Nav2 blocker.

**Effort:** ~15 min once `g1_write_node` is up and the operating envelope
is safe.

---

### R-008 — `g1_write_node` destructor cyclone DDS assertion (intermittent)
**Symptom:** On clean SIGINT shutdown, ~25% of runs (1/4 observed) trigger:
```
cyclonedds-cxx-0.10.2/src/ddscxx/src/org/eclipse/cyclonedds/core/EntityDelegate.cpp:255:
EntityDelegate::prevent_callbacks(): Assertion `false' failed.
```
followed by `Aborted (core dumped)`. The other 3/4 destructors run cleanly
("Destructor: node destroyed cleanly").

**Impact:** Runtime is unaffected — the assertion is in the **destructor**
path, after motion has stopped. Process exit becomes abnormal (core dump,
non-zero exit code). Restart loops or watchdogs may misread this as a
runtime failure.

**Hypotheses:**
- Race between `client_.reset()` (SDK destruction) and CycloneDDS
  participant teardown.
- `~G1Write()` resets ROS subscriptions before `g1_driver_`, so SDK threads
  may still be calling into already-destroyed CycloneDDS entities.
- The duplicate `cmd_vel_subscription_.reset()` at `g1_write.cpp:25-26`
  is benign but smells of unfinished cleanup logic.

**Approach:**
1. Run `g1_write_node` under `gdb`, raise the SIGABRT, capture backtrace.
2. Try reordering `~G1Write()` to reset `g1_driver_` first, then the ROS
   entities.
3. Try adding a small sleep / explicit `client_->Stop()` before
   `client_.reset()`.

**Effort:** ~1–2 hours; low priority because it doesn't affect operation.

## Done

### 2026-05-26 — R-003 closed: G1 walks under `g1_write_node` (motion verified)
**Errata for the original R-003 description:** the symptom was *not*
`free(): invalid pointer` in `ChannelFactory::Init(0)`. That report was
based on a corrupted observation — a diagnostic command had wrapped
the binary in `timeout 8`, which sent `SIGTERM` mid-`on_configure`,
invalidating the rclcpp context and causing `create_publisher` to fail
afterwards. The cascade was misread as an SDK-level crash.

**Actual behavior, verified in clean runs:**
- `g1_write_node` binary launches into `unconfigured` cleanly.
- `lifecycle set configure` → `Transitioning successful` (SDK init
  works: `ChannelFactory::Init(0)` + `LocoClient::Init()` +
  `lowcmd_publisher->InitChannel()` + `lowstate_subscriber->InitChannel()`).
- `lifecycle set activate` → `active [3]`.
- `ros2 topic pub --once /cmd_vel_out geometry_msgs/msg/Twist '{linear:
  {x: 0.5}}'` → G1 takes a step.

**Two new findings split off as separate Roadmap items:**
- **R-007** — single-step dead-zone characterisation (reframed as
  characterisation, not Nav2 blocker; see D-011 v3).
- **R-008** — destructor-side cyclone DDS assertion `prevent_callbacks()`
  fires intermittently (~25%) on clean SIGINT. Doesn't affect runtime.

**Consequence for D-010 / D-011:** D-010's premise (defer motion because
`g1_write_node` crashes) is invalidated. D-010 marked superseded by
D-011, which carries the motion-enablement decision and the (revised)
take on the velocity dead-zone.

### 2026-05-25 — Nav2 stack merged into `3d_nav_ros2` container
Verified end-to-end: goal → plan → cmd_vel_out at 10 Hz. ADRs D-008 / D-009
/ D-010. Test report at `docs/TEST_REPORTS/2026-05-25-nav2-pipeline.md`.

### 2026-05-25 — Cross-host RViz2 + 2D Goal Pose tool
Added `rviz_default_plugins/SetGoal` and `PublishPoint` to
`configs/g1_track0_rviz2.rviz`. Verified Leo-end goal publication reaches
G1 over Zenoh.

### 2026-05-25 — `/map` PointCloud2 visible in RViz2
ADR D-004 fixed the publisher QoS (TRANSIENT_LOCAL). Subscriber side fixed
in `configs/g1_track0_rviz2.rviz` to match Durability Policy.

### 2026-05-25 — `pointcloud_to_laserscan` parameter parity with ROS 1
Mirrored ROS 1's parameters in `launch.sh` (D-006).

### 2026-05-25 — fast_lio long-run drift (operationally addressed)
Initially reported as a 5-minute ESKF divergence; root cause was missing
initial pose. Documented in ADR D-007 and `~/.claude/.../memory/
g1_fastlio_initial_pose.md`. With correct initial pose, 30+ minute static
runs are stable.

### 2026-05-25 — `g1_3d_nav_ros2` repo created and full source pushed
Commit `498e3af` (initial full tree + cleanup). Pushed from G1 over the
Leo HTTP proxy via SSH deploy key.
