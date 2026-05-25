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

### R-003 — Fix `g1_write_node` SDK initialization crash
**Symptom:** `g1_write_node` is intentionally not started today (ADR D-010).
When started manually, it crashes during `ChannelFactory::Init(0)` with
`free(): invalid pointer`. With `g1_write_node` down, the entire chain ends
at `/cmd_vel_out`; the robot does not move.

**Hypotheses to check:**
- ABI mismatch between Unitree SDK in `/opt/robot_sdk/lib` (host
  bind-mount) and the C++ standard library inside the container.
- Double-free in `g1_driver.cpp` early init path.
- DDS / network setup race when the SDK probes for `eth0`.

**Approach:**
1. Run `g1_write_node` under `gdb` inside the container, capture backtrace
   at SIGABRT.
2. If ABI mismatch, build `g1_pkg` against the in-container libstdc++ and
   compare.
3. If clean, start with a small wrapper that just calls `ChannelFactory::Init`
   and prints; bisect from there.

**Effort:** unknown, blocked on container reproduction.

**Once unblocked:** add a `[3/3] g1_write_node` step to `nav2_launch.sh`
behind an opt-in flag (`ENABLE_MOTION=1`), so the default still keeps G1
still.

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

**Decision needed:** When G1 motion is enabled (after R-003), do we add an
explicit publisher for `/dead_man_switch` and require a Leo-side button or
gamepad press to release it? Or accept the current "no lock without
publisher" behavior?

This is operationally important and will be revisited as part of R-003
unblocking.

---

### R-006 — Upstream contributions to botbrain
Once R-001 and R-003 are settled, consider PRs back to botbrain:
- `nav2_params.yaml` topic indirection (parameterise `static_layer.map_topic`
  and `obstacle_layer.cloud.topic` instead of hardcoding) so external
  consumers don't need a fork.
- Whatever turns out to be the reliable lifecycle-activation pattern in
  rmw_zenoh_cpp deployments.

## Done

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
