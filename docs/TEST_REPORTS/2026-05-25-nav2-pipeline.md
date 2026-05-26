# Test Report — Nav2 software pipeline (2026-05-25)

> Verification of the data pipeline introduced by ADRs D-008 / D-009 / D-010
> (Nav2 merged into `3d_nav_ros2`, software-only PoC). G1 does not move.

## Setup at test time

- Container: `3d_nav_ros2` running image `g1_nav_final:latest` SHA `137a5b46be62`
- All 6 localization steps OK (`launch.sh`)
- Nav2 stack started (`nav2_launch.sh`); 6 lifecycle nodes active [3]:
  controller_server, planner_server, bt_navigator, behavior_server,
  smoother_server, waypoint_follower
- twist_mux + lifecycle_manager_navigation running
- Initial pose was set in advance, ICP confidence > 0.7
- A goal was sent from Leo RViz2 via the new `2D Goal Pose` tool

## Tracer results

| # | Topic / Signal | Test | PASS evidence |
|---|---|---|---|
| 1 | `/plan` | `ros2 topic echo /plan --once` after `/goal_pose` published | `nav_msgs/msg/Path` with `frame_id: map`, multiple `poses` with metric x/y values |
| 2 | `/cmd_vel_nav` | `ros2 topic hz /cmd_vel_nav` | **20.06 Hz** average (matches `controller_frequency: 20.0` in `nav2_params.yaml`); `Twist` carries non-zero linear.x and angular.z |
| 3 | `/cmd_vel_out` | `ros2 topic hz /cmd_vel_out` after `zero_vel_publisher activate` | **9.99 Hz** average (matches `twist_mux` output rate); `Twist` reflects `/cmd_vel_nav` content |
| 4 | TF / lifecycle / odom | `ros2 lifecycle get` for each Nav2 node, `ros2 topic hz /Odometry_loc` | All 6 Nav2 lifecycle nodes report `active [3]`; `/Odometry_loc` 10.00 Hz |

**End-to-end software loop is GREEN.** Goal → plan → control command → twist_mux → final output.

## Pipeline as actually wired

```
RViz "2D Goal Pose"
     │
     ▼ /goal_pose
bt_navigator
     │  (action calls)
     ├──▶ planner_server ──▶ /plan          (nav_msgs/Path)
     ├──▶ controller_server ──▶ /cmd_vel_nav   (Twist, 20 Hz)  ──┐
     │                                                           ▼
     ├──▶ behavior_server ──▶ /cmd_vel  (4 publishers — recovery)│
     │                            spin / back_up /                │
     │                            drive_on_heading / wait        │
     ▼                                                            ▼
  /cmd_vel_zero (1Hz, priority 1)  ───────────────────────▶  twist_mux
                                                                  │
  (other inputs: cmd_vel_rosa, manipulation_vel,                  │
   cmd_vel_nipple, cmd_vel_joy)                                   │
                                                                  ▼
                                                          /cmd_vel_out  (10 Hz)
                                                                  │
                                                                  ▼
                                                  (g1_write_node, NOT started in PoC)
```

Note: `/cmd_vel` is **not** the final controller-output topic, despite the
name. It is one of multiple twist_mux input streams (the Nav2 `behavior_server`
recovery channel, priority 10 in `twist_mux.yaml`). The final command consumed
by `g1_write_node` is **`/cmd_vel_out`**.

## Non-blocking findings — added to docs/ROADMAP.md

### A. `/cmd_vel` has 4 simultaneous publishers from `behavior_server`
Nav2's recovery server registers one publisher per behavior plugin (spin /
back_up / drive_on_heading / wait). All four publish to the same `/cmd_vel`
topic. This was previously misread as a "twist_mux output"; it is not.
twist_mux subscribes to `cmd_vel_nav`, not `cmd_vel`, and outputs to
`cmd_vel_out`. ADR D-008/D-010 + README updated to reflect this.

### B. `zero_vel_publisher` is `unconfigured` after auto-launch
`bot_bringup/twist_mux.launch.py` registers `OnProcessStart` →
`TRANSITION_CONFIGURE` for `zero_vel_publisher`, but in this rmw_zenoh_cpp
deployment the lifecycle event handler does not deliver, leaving the node
permanently `unconfigured`. Manual recovery:

```bash
ros2 lifecycle set /zero_vel_publisher configure
ros2 lifecycle set /zero_vel_publisher activate
```

After activate, `/cmd_vel_zero` starts publishing zero `Twist` at low rate
(priority 1 fallback in `twist_mux.yaml`), which keeps `/cmd_vel_out` alive
even when no higher-priority source is present. Without this, when no
`/cmd_vel_nav` is being published `/cmd_vel_out` goes silent.

This will be fixed by `nav2_launch.sh` adding an explicit
`ros2 lifecycle set` step after launch (Roadmap R-001).

### C. Container-internal `ros2 cli` intermittently `!rclpy.ok()`
Symptom: `ros2 topic hz`, `ros2 topic echo` inside the container return
`failed to initialize wait set: the given context is not valid` after a few
invocations, even though pubs/subs continue to work fine.

Workaround: invoke `ros2 cli` from Leo (which connects to the same Zenoh
router); Leo-side cli is stable. Or restart `ros2 daemon`:

```bash
ros2 daemon stop && ros2 daemon start && sleep 3
```

Origin: rmw_zenoh_cpp 0.1.8 daemon implementation. Tracked in Roadmap R-002.

## Reproduce

```bash
# G1 host, after launch.sh + nav2_launch.sh have run for ~2 minutes
ssh unitree@192.168.100.30 'docker cp ~/g1_3d_nav_ros2_repo/tools/tracer_nav2_pipeline.sh 3d_nav_ros2:/tmp/'
ssh unitree@192.168.100.30 'docker exec 3d_nav_ros2 bash /tmp/tracer_nav2_pipeline.sh'
```

(Full tracer script is in `tools/tracer_nav2_pipeline.sh`.)

## What still does not work / out of scope

- **G1 does not move.** This was decided in ADR D-010. `g1_write_node` is not
  started in `nav2_launch.sh` because of the `ChannelFactory::Init(0)` →
  `free(): invalid pointer` SDK-level crash. Fixing that is Roadmap R-003.
- **Map → costmap pipeline not stress-tested.** Static layer subscribed to
  `/map_2d` (per fork in `config/nav2_params.yaml`); single goal succeeded,
  but obstacle-avoidance under live `/scan` updates was not verified.
  Roadmap R-004.
