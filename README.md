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
│ /g1_3d_nav_ros2/maps -> /home/unitree/.../maps │    └────────────────┘
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
2. Place `scans.pcd` at `/home/unitree/g1_3d_nav_ros2_repo/maps/scans.pcd` (see
   `maps/README.md` for how to obtain).
3. Create the runtime container if it does not exist:
   ```bash
   docker run -d --name 3d_nav_ros2 \
       --network host --ipc host \
       -v /home/unitree/g1_3d_nav_ros2_repo/maps:/g1_3d_nav_ros2/maps \
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

1. **Manually re-position G1** at a known origin and restart `bash /g1_3d_nav_ros2/tools/launch.sh`.
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
docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/launch.sh
# wait for "=== ALL 6 NODES RUNNING ==="

# Window B — Nav2 + twist_mux + g1_write_node (holds session)
ssh unitree@192.168.100.30
docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/nav2_launch.sh
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

Both wrappers live in the canon repo at `tools/soft_stop.sh` and
`tools/estop.sh`. The repo is bind-mounted into the container at
`/g1_3d_nav_ros2/`, so the operator-side invocation is single-line:

```bash
docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/soft_stop.sh   # routine
docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/estop.sh       # emergency (squats)
```

The wrappers self-source ROS env and `RMW_IMPLEMENTATION=rmw_zenoh_cpp`
so they work regardless of the caller shell. Because the repo is
bind-mounted, `git pull` on the G1 host immediately makes any
updated tool available inside the container — no `docker cp`.

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

## Mapping

The ROS2-native mapping pipeline. Use this when you need to (re)build
the maps for a new environment or to fix ICP-fitness-low ("the robot
drifts after walking around") symptoms in the existing one.

**What you get out of it:** two files written into
`/g1_3d_nav_ros2/maps/` (== host `/home/unitree/g1_3d_nav_ros2_repo/maps/`,
the canon repo working tree):

- `scans.pcd` — 3D point cloud, used by `open3d_loc` for ICP localization
- `accumulated_grid.pgm` + `.yaml` — 2D occupancy grid, used by nav2's
  static layer

`mapping_save.sh` also writes `.bak` siblings of the previous maps for
one-step rollback.

### Prerequisites

- The `3d_nav_ros2` container exists and uses the new mount layout
  (`/home/unitree/g1_3d_nav_ros2_repo/maps:/g1_3d_nav_ros2/maps`).
  If you're migrating from the old `/home/unitree/g1_3d_nav/maps`
  mount or from the older `/root/maps` layout, do this once:
  ```bash
  ssh unitree@192.168.100.30
  cp /home/unitree/g1_3d_nav/maps/scans.pcd \
     /home/unitree/g1_3d_nav_ros2_repo/maps/scans.pcd
  cd /home/unitree/g1_3d_nav_ros2_repo && git pull
  docker stop 3d_nav_ros2 && docker rm 3d_nav_ros2
  bash tools/recreate_3d_nav_ros2.sh
  ```
- The repo is bind-mounted into the container at `/g1_3d_nav_ros2/`,
  so the mapping wrappers (`tools/mapping/mapping_record.sh`,
  `tools/mapping/mapping_save.sh`, `tools/mapping/grid_accumulator.py`)
  are immediately available inside the container after `git pull`. No
  `docker cp` step.

### Step 1 — bring up the localization stack

`fast_lio` runs in mapping mode by default; it accumulates the 3D PCD as
it goes, and dumps it on `/map_save` service call.

```bash
# window A — hold this session open the whole time
ssh unitree@192.168.100.30
docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/launch.sh
```

Wait for `=== ALL 6 NODES RUNNING ===`. Then wait another ~10 seconds
for `IMU Initial Done` to appear in `/tmp/fastlio.log` — during this
window G1 must be standing still (sport mode, no walking yet).

**Pick the spot where G1 is standing right now** as your map origin.
The 2D grid's origin and the 3D PCD's frame zero will both be tied to
this physical position, so make sure it's somewhere you can return to
later if you ever need to align an old map with a new mapping run.
Floor tape works.

### Step 2 — start the 2D grid accumulator

```bash
# window B
ssh unitree@192.168.100.30
docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/mapping/mapping_record.sh
```

You should see:

```
grid_accumulator: res=0.05m, ground_z<0.15, obstacle_z>0.25, ...
```

and every 5 seconds a stats line: `frames=N ground=N obs=N grid=WxH`.

This window blocks while G1 is being driven around — leave it open.

### Step 3 — drive G1 around the workspace

Use the RC controller, sport mode, slow walk. **The single
viewpoint of a stationary scan has blind spots — driving around is
how those get filled.** The accuracy of the rest of your testing
depends on this step.

Recommended motion pattern:

- Cover every place you'll later put a waypoint
- Pass through every doorway / narrow corridor from both directions
- At each future-waypoint location, **stop for ~5 seconds** to let
  fast_lio accumulate dense local geometry there
- Take some "facing rotations" in place — turn slowly so the LiDAR
  sweeps the surroundings from each viewpoint
- 5 to 15 minutes total is typical; longer is fine, shorter risks
  blind spots

When done, **drive G1 back to the floor-tape origin** (Step 1 spot).
This makes the post-map-restart alignment trivial.

### Step 4 — dump the maps

```bash
# window C
ssh unitree@192.168.100.30
docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/mapping/mapping_save.sh
```

Expected output:

```
[1/5] dumping 3D PCD via fast_lio /map_save...
  ... success: True
  found new PCD at /root/test.pcd
[2/5] dumping 2D PGM via map_saver_cli...
  ... [INFO]: Map saved successfully
[3/5] backing up previous maps to .bak...
[4/5] moving new maps into /g1_3d_nav_ros2/maps/ ...
[5/5] fixing yaml image path...

DONE. New map files in /g1_3d_nav_ros2/maps/ :
  -rw-r--r-- 1 root root 258123456 ... scans.pcd
  -rw-r--r-- 1 root root   1234567 ... accumulated_grid.pgm
  -rw-r--r-- 1 root root       154 ... accumulated_grid.yaml

Next:
  1. Ctrl+C window A's launch.sh
  2. Re-run /g1_3d_nav_ros2/tools/launch.sh — open3d_loc loads new PCD on startup
  3. Verify ICP fitness >= 0.7
```

If you don't see "DONE." or any step says FAIL, **don't restart the
stack** — your old map is still in place and still working. See the
troubleshooting block below.

### Step 5 — restart the stack with the new maps

```bash
# window A
# Ctrl+C the running launch.sh
docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/launch.sh
```

`open3d_loc` reads `scans.pcd` once at startup, so the restart is
required for the new PCD to take effect. `map_server` reads
`accumulated_grid.{pgm,yaml}` once at startup too.

### Step 6 — verify ICP fitness ≥ 0.7

```bash
ssh unitree@192.168.100.30 \
  'docker exec 3d_nav_ros2 tail -50 /tmp/loc.log | grep fitness | tail -10'
```

Expected:

```
reg_result.fitness: 0.83  ...
reg_result.fitness: 0.79  ...
```

ICP fitness above 0.7 is the threshold open3d_loc uses to decide
whether to publish a `map → odom` correction. **If your fitness
stays below 0.7, the new map is not going to work** — the rest of
the navigation chain depends on this. Skip to the troubleshooting
block, do not start any waypoint test.

### Troubleshooting mapping

| Symptom | Likely cause | Fix |
|---|---|---|
| `mapping_save.sh` says "fast_lio did not produce a non-empty test.pcd" | `pcd_save_en: false` in mid360.yaml, or fast_lio has been running for <30 s | Check yaml; let it run longer before saving |
| `mapping_save.sh` says "no map saver service" | `mapping_record.sh` is not running, or grid_accumulator hasn't published `/accumulated_grid` yet | Start `mapping_record.sh` in window B; let it accumulate at least one frame |
| Fitness stays at 0.0 after restart | The PCD was dumped while G1 had drifted — fast_lio's odom != map origin | Re-run mapping starting from a fresh `docker stop && start`; G1 stays still until "IMU Initial Done"; finish back at the start position |
| Fitness oscillates 0.3–0.6 | Workspace was scanned from too few viewpoints | Re-run mapping with more rotations + back-and-forth coverage |
| New map breaks something but old map worked | Roll back: `cd /g1_3d_nav_ros2/maps/ && for f in scans.pcd accumulated_grid.pgm accumulated_grid.yaml; do mv "$f.bak" "$f"; done`, then restart launch.sh | — |

### Versioning the new PGM in canon (optional)

The PGM/yaml landed in the canon repo's working tree. To capture
the change in git:

```bash
# Leo side
cd <your canon clone>
ssh unitree@192.168.100.30 'cat /home/unitree/g1_3d_nav_ros2_repo/maps/accumulated_grid.pgm'  > maps/accumulated_grid.pgm
ssh unitree@192.168.100.30 'cat /home/unitree/g1_3d_nav_ros2_repo/maps/accumulated_grid.yaml' > maps/accumulated_grid.yaml
git add maps/accumulated_grid.* && git commit -m "maps: re-mapped <date>" && git push
```

`scans.pcd` stays out of git regardless — it's per-environment and
too large.

## Gotop — capture waypoints + navigate to one

This section covers the testing toolkit for "drive G1 to a named spot
and check whether nav2 actually got it there." Three tools, all in
`tools/`, all label-driven (no fixed point count).

### Tool overview

| Tool | What it does | When to use |
|---|---|---|
| `tools/capture_waypoints.py` | REPL for marking the current G1 pose under a label | After mapping, walk G1 to each spot you care about |
| `tools/goto_waypoint.py` | REPL for sending G1 to one waypoint at a time, with achieved-vs-goal error reporting and an append-only CSV history | "Send G1 to kitchen and look at the result"; ad-hoc inspection |
| `tools/navigate_batch.py` | Batch script — visits a list of labels × N rounds, writes a markdown accuracy report | Statistical accuracy testing across many waypoints |

All three tools:

- Read/write a single YAML file (`waypoints.yaml` by default)
- Use **labels** as identifiers — pick any string matching
  `[A-Za-z][A-Za-z0-9_-]*` (e.g. `kitchen`, `door1`, `lab_corner_3`)
- No fixed count of waypoints — capture as many as you want, repeat
  labels overwrite, name doesn't have to start with `wp`
- Self-source ROS env + RMW config; `docker exec` invocation is one line
- Probe `map→body` TF and (for goto / navigate_batch) the
  `/navigate_to_pose` action server at startup, with explicit
  `is launch.sh running?` / `is nav2_launch.sh running?` hints on
  timeout

### Where the tools live (no docker cp needed)

The Gotop tools are at `tools/gotop/` in this repo. The brake
wrappers (`soft_stop.sh`, `estop.sh`) are at the top of `tools/`.
Because the repo is bind-mounted into the container at
`/g1_3d_nav_ros2/`, every tool is immediately invocable as
`/g1_3d_nav_ros2/tools/gotop/<name>` (or
`/g1_3d_nav_ros2/tools/<brake>` for the brakes). After `git pull`
on the host, new versions are available inside the container with
no `docker cp`.

### Capturing waypoints

Need: `launch.sh` running. (`nav2_launch.sh` not required for capture.)

```bash
docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/gotop/capture.sh
```

The wrapper sources ROS env, points the underlying python at the
default yaml `/g1_3d_nav_ros2/data/waypoints.yaml` (persistent —
survives container stop/start because `/g1_3d_nav_ros2/data/` is
the bind-mounted host repo working tree). Override with
`env WAYPOINTS_YAML=/some/path.yaml ...` if you need a separate
captures file.

You'll see:

```
Checking map->body TF stream... OK

Loaded N existing waypoints from /g1_3d_nav_ros2/data/waypoints.yaml:
  kitchen          x=  1.230  y=  4.560  yaw=  90.0deg
  door1            x=  7.890  y=  1.010  yaw=   0.0deg
  ...

Commands at prompt:
  <label>            capture current pose under that label
  list / ls          show all captured waypoints
  del <label>        delete a waypoint
  rename <old> <new> rename a waypoint (group refs updated too)
  q / quit           save and exit (Ctrl-D works too)
```

Then:

1. Use the RC controller to drive G1 to where you want a waypoint
2. Wait for G1 to stand still (sport-mode micro-jitter takes ~1 second
   to settle)
3. At the prompt type a label and press Enter
4. The script samples the `map→body` transform at 30 Hz for 1 second,
   averages it, writes the entry to the YAML
5. Repeat. Type `q` when done.

The YAML is rewritten after **every** capture, so Ctrl-C never loses
prior data. Restarting `capture_waypoints.py` against the same YAML
loads existing waypoints and lets you keep adding — incremental
sessions are fully supported.

#### Re-capturing an existing label

Type the same label again — the script samples a fresh pose, then:

- If the new pose is **within 30 cm** of the old one → silent refresh
  (small drift is expected, no point asking)
- If the new pose is **more than 30 cm** away → confirmation prompt:

  ```
  'kitchen' exists at (1.23, 4.56); new is (5.20, 1.40), 4.45m away.
  Overwrite? (y/N):
  ```

  Default `N` keeps the old value. Useful safety against typing the
  wrong label.

#### Optional: groups

YAML can carry a `groups:` section that `navigate_batch.py` understands
via `@groupname` tokens. Hand-edit the YAML to add it:

```yaml
frame_id: map
waypoints:
  kitchen:      {x: 1.23, ...}
  kitchen_door: {x: 1.50, ...}
  lab_corner:   {x: 5.00, ...}
groups:
  kitchen_zone: [kitchen, kitchen_door]
  lab:          [lab_corner]
```

`capture_waypoints.py` preserves the `groups:` section on save and
keeps it consistent on `del` / `rename` — so you can edit groups
once and the capture tool won't clobber them.

### Driving G1 to a single waypoint (interactive)

Need: `launch.sh` **and** `nav2_launch.sh` running, plus a `waypoints.yaml`.
Operator on site, RC controller in hand (D-011 safety preconditions).

```bash
docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/gotop/goto.sh
```

The wrapper points the script at the default yaml
`/g1_3d_nav_ros2/data/waypoints.yaml` and the default CSV history
`/g1_3d_nav_ros2/data/goto_history.csv`. Both persist across container
stop/start. Override via `WAYPOINTS_YAML=/some/path.yaml ...` or by
appending `--csv /some/path.csv` to the command.

Prompt:

```
goto> kit<TAB>             ← tab completion on labels
goto> kitchen              ← G1 walks; nav2 status + xy/yaw error printed
goto> list                 ← list all available labels
goto> q                    ← exit cleanly
goto> q!                   ← cancel current motion (zero-vel) + exit
^C                          ← while G1 is moving = same as q!
```

What the **soft stop** (Ctrl-C / `q!`) actually does, in order:

1. Cancel the in-flight `/navigate_to_pose` goal — nav2 stops
   publishing `/cmd_vel_nav`
2. `twist_mux` 0.2 s timeout falls back to `/cmd_vel_zero` (priority 1,
   the zero_vel_publisher's 0 Twist)
3. `g1_write_node` receives the 0 Twist, calls SDK `Move(0, 0, 0)`
4. **G1 stops in place, still standing in sport mode**

Crucially: **no FSM transition, no squat**. To recover G1 stays
standing and is immediately ready for the next goal — that's the
point of the soft stop, vs `estop.sh` which fail-passively squats G1.

#### CSV history (automatic)

Every segment, success or failure, appends a row to
`/g1_3d_nav_ros2/data/goto_history.csv`:

```csv
timestamp,label,goal_x,goal_y,goal_yaw_deg,nav2_status,duration_s,reached_x,reached_y,reached_yaw_deg,xy_err_m,yaw_err_deg
2026-05-26T10:30:15,kitchen,1.2300,4.5600,90.00,SUCCEEDED,12.30,1.2400,4.5500,89.50,0.0141,-0.50
2026-05-26T10:32:01,door1,7.8900,1.0100,0.00,USER_CANCELED,5.20,,,,,
```

Override path with `--csv /custom/path.csv`. The file is
append-only, so multiple sessions accumulate naturally; if you want
a fresh history just `rm` the file first.

### Batch accuracy testing (statistical)

Need: `launch.sh` + `nav2_launch.sh` running, plus `waypoints.yaml`.
Operator on site, ready to answer `physical_sanity (y/n/skip)` after
each segment.

```bash
# everything in the yaml × 3 rounds, randomise order each round
docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/gotop/batch.sh \
    --all --rounds 3 --shuffle

# specific labels
docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/gotop/batch.sh \
    --labels kitchen,door1,lab_corner --rounds 3

# by group (yaml has a groups: section)
docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/gotop/batch.sh \
    --labels @kitchen_zone --rounds 3

# mix labels and groups (auto-deduped)
docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/gotop/batch.sh \
    --labels @kitchen_zone,@lab,door1 --rounds 3 --shuffle
```

The wrapper points the script at default yaml
`/g1_3d_nav_ros2/data/waypoints.yaml` and writes the report to
`/g1_3d_nav_ros2/data/batch_report.md` (each run overwrites — copy
to a dated name if you want to keep history).

The script writes a markdown report at `/g1_3d_nav_ros2/data/batch_report.md` with:

- one row per segment: round, label, goal pose, reached pose,
  xy_err, yaw_err, duration, nav2 status, sanity
- per-label summary: success rate, mean ± std xy_err, mean ± std
  yaw_err across all rounds

Override the report path with `--output /custom/report.md`.

### Brakes — when goto / batch is running

If you need to stop G1 while a script is running:

| Way | Effect | When |
|---|---|---|
| **Ctrl-C** in the goto/batch terminal | Soft stop (cancel goal, zero-vel, G1 standing). Script exits | Most common — "wrong target, change my mind" |
| **`docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/soft_stop.sh`** in another terminal | Same soft stop, but doesn't exit the goto/batch script | Hands aren't on the goto terminal |
| **`docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/estop.sh`** | Hard stop + squat (toggle: 2nd call to undo) | Real emergency, G1 about to fall / hit something |
| **RC controller L2+B** | Hardware brake, independent of ROS | Always-available fallback |

The two scripted brakes are documented in detail in the
"Two in-stack brakes" subsection of "Daily operation" above.

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
