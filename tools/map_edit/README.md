# Map editing (operator workstation)

After a mapping run produces `maps/accumulated_grid.{pgm,yaml}` (see the
top-level `README.md` → `## Mapping`), you'll often need to clean up the
result by hand: erase ghost obstacles, paint virtual walls across no-go
zones, mark named regions. None of that is automated — it's GUI work
done at an operator's workstation, not on G1.

This directory is a self-contained ROS 1 noetic editor. It includes:

- `Dockerfile` — operator-side image (`ros:noetic-ros-base` + RViz +
  the vendored `ros_map_edit` package built into `/catkin_ws/`)
- `start_map_edit.sh` — launcher that handles X11 forwarding, plugin
  discovery, and clearing inherited `ROS_MASTER_URI` so the local
  roslaunch starts its own roscore
- `ros_map_edit/` — vendored RViz panel + 4 RViz tools, **patched**
  (see `## Patches applied to vendored ros_map_edit` below)

The G1 itself never uses this image. The flow is:

```
G1 produces map  ──scp──►  workstation edits  ──scp──►  G1 reloads
                          (this directory)
```

## Build the editor container (one-off)

On the workstation:

```bash
cd <repo>/tools/map_edit
docker build -t map_edit_rviz:latest .

# Create the container with maps mounted and X11 forwarding set up.
# Adjust $HOME/g1_maps to wherever you want edited maps to land.
mkdir -p "$HOME/g1_maps"
docker run -d --name map_edit_rviz \
    -e DISPLAY="$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$HOME/g1_maps:/root/maps" \
    map_edit_rviz:latest
```

The container holds `sleep infinity`; the launcher uses `docker exec`.

If your existing setup uses different names or paths, override at
launch time (see `## Configuration reference` below).

## Pull the latest map from G1

```bash
G1=unitree@<g1-host>          # e.g. unitree@100.78.247.66
H="$HOME/g1_maps"

scp "$G1":/home/unitree/g1_3d_nav_ros2_repo/maps/accumulated_grid.{pgm,yaml} "$H/"

# G1's yaml uses an absolute container path that won't resolve on the
# workstation. Rewrite to a relative path.
sed -i 's|^image:.*|image: accumulated_grid.pgm|' "$H/accumulated_grid.yaml"
```

## Launch the editor

```bash
./start_map_edit.sh /root/maps/accumulated_grid.yaml
```

RViz comes up. Left panel shows **File Management** with the loaded
map, a green **Save All Files** button, and an **Open Map** button.
Toolbar gains four tools: `MapEdit`, `VirtualWall`, `Region`,
`MapEraser`.

If the panel does not appear, see `## Known issues` below.

## Edit

| Tool | What it does | Mouse |
|---|---|---|
| **MapEraser** | Repaint cells: change OCCUPIED ↔ FREE ↔ UNKNOWN | LMB: black (occupied), RMB: white (free), drag for continuous strokes |
| **VirtualWall** | Add a 2-point line wall — published as overlay, saved to `<map>.json` | LMB: click two endpoints, RMB: cancel current wall |
| **Region** | Mark a named polygon region — saved to `<map>_region.json` | LMB: click polygon vertices, double-click to close |
| **MapEdit** | Mode switcher; selects which of the above is active | — |

Brush size, virtual wall color/width, etc. are exposed in the right-side
**Tool Properties** panel when each tool is selected.

## Save

Click **Save All Files** (green button in the File Management panel).
Four files get written **next to the loaded yaml**, basename preserved,
existing files overwritten:

- `accumulated_grid.yaml` — map config
- `accumulated_grid.pgm` — map image (your edits applied)
- `accumulated_grid.json` — virtual walls (empty `{"vws": []}` if none drawn)
- `accumulated_grid_region.json` — regions (empty `{"regions": []}` if none drawn)

A confirmation dialog summarises what was saved. The host directory
mounted into the container (`$HOME/g1_maps` in the example above) sees
the new files immediately.

## Deploy edits back to G1

```bash
G1=unitree@<g1-host>
H="$HOME/g1_maps"

# 1. Re-add `mode: trinary`. The editor drops it on save; nav2 defaults
#    to trinary so it would still work, but we've been bitten enough by
#    "default values change" to be explicit.
grep -q '^mode:' "$H/accumulated_grid.yaml" \
  || sed -i '2a mode: trinary' "$H/accumulated_grid.yaml"

# 2. Switch image: back to G1-side absolute container path.
sed -i 's|^image:.*|image: /g1_3d_nav_ros2/maps/accumulated_grid.pgm|' \
    "$H/accumulated_grid.yaml"

# 3. scp four files (region.json may not exist if you didn't draw any
#    regions — that's fine, glob will skip it).
scp "$H"/accumulated_grid.{pgm,yaml,json}        "$G1":/home/unitree/g1_3d_nav_ros2_repo/maps/
scp "$H"/accumulated_grid_region.json 2>/dev/null "$G1":/home/unitree/g1_3d_nav_ros2_repo/maps/ || true

# 4. Hot-reload G1's map_server — no nav-stack restart, no fast_lio re-init.
ssh "$G1" 'docker exec 3d_nav_ros2 bash -c "
    source /opt/ros/humble/setup.bash; source /botbrain_ws/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_CONFIG_OVERRIDE=mode=\"client\";connect/endpoints=[\"tcp/127.0.0.1:7448\"]
    ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \
        \"{map_url: '\''/g1_3d_nav_ros2/maps/accumulated_grid.yaml'\''}\"
"'
```

To verify the reload took effect, on G1:

```bash
docker exec 3d_nav_ros2 bash -c '
    source /opt/ros/humble/setup.bash; source /botbrain_ws/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/127.0.0.1:7448\"]"
    timeout 5 ros2 topic echo /map_2d --once --field info
'
```

`map_load_time` in the response should be the timestamp of the reload.

## Patches applied to vendored ros_map_edit

The `ros_map_edit/` source in this directory is **not pristine
upstream** — it has been patched. The Dockerfile builds the patched
source as-is; no separate patch step is needed.

1. **`getCurrentMapFile()` falls back to `/rviz/map_file`**
   (`src/map_edit_panel.cpp`, `src/region_tool.cpp`,
   `src/virtual_wall_tool.cpp`). Stock behaviour only checks
   `/map_server/map_file`, but ROS 1 `map_server` consumes its private
   params at startup so that key isn't visible at runtime. The
   shipped `launch/map_edit.launch` already sets `/rviz/map_file`, so
   the fallback recovers the path with no operator action. Without
   this patch, `Save All Files` always errors with "Please load a map
   file first" until the operator manually clicks `Open Map`.

2. **All user-visible strings are English**
   (`src/*.cpp`, panel labels, tool status messages, dialog titles).
   Stock strings are UTF-8 Chinese; under most Qt locales those render
   as mojibake squares. Internal code comments are still in their
   original language; they don't enter the binary.

If you re-vendor a fresh upstream `ros_map_edit` checkout, both
patches must be re-applied or the operator-facing UX regresses.

## Known issues

- **`free_thresh: 0.196` is load-bearing — do not let any tool reset
  it to nav2's default 0.25.** `tools/mapping/mapping_save.sh` already
  passes `--free 0.196 --occ 0.65 --mode trinary` to `map_saver_cli`,
  and `ros_map_edit` preserves the field on round-trip. Reason: the
  mapping pipeline encodes UNKNOWN cells as PGM pixel value 205, which
  `map_server` translates to occupancy probability `(255-205)/255 =
  0.196`. With the default 0.25 threshold, those UNKNOWN cells get
  reclassified as FREE — the navigable area becomes "everywhere," and
  the planner happily routes through walls.

- **`mode: trinary` is dropped on save.** The deploy step rewrites it
  back. Nav2's default still happens to be trinary, but the assumption
  is fragile.

- **Plugin panel does not appear in RViz.** Means
  `/catkin_ws/devel/setup.bash` was not sourced before `roslaunch` —
  `ROS_PACKAGE_PATH` doesn't include the workspace, so RViz silently
  drops the unknown plugin. `start_map_edit.sh` handles this; if you
  launch RViz by hand, source the workspace first.

- **`Save All Files` does nothing / silently fails.** Inspect
  `~/.ros/log/latest/rosout.log` (inside the container, in the
  container's `$HOME`); the error path uses `ROS_ERROR` so it lands
  there. Most common cause is permission (the mounted maps dir is
  owned by host UID, container `root` may or may not write to it
  depending on Docker storage driver).

- **No nav2 / no master.** `roslaunch` starts a fresh local roscore
  for editing. Do not point `ROS_MASTER_URI` at G1 — the editor
  publishes/subscribes only to its local ROS graph; trying to share
  the graph with G1 will fight with G1's own ROS 2 stack.

## Configuration reference

`start_map_edit.sh` reads three environment variables:

| Var | Default | Meaning |
|---|---|---|
| `CONTAINER` | `map_edit_rviz` | docker container name |
| `CATKIN_WS` | `/catkin_ws` | workspace path inside container |
| `MAPS_IN_CONT` | `/root/maps` | mounted maps dir inside container |

Common overrides:

```bash
# An already-existing container with a different name and path layout:
CONTAINER=my_existing_rviz CATKIN_WS=/tmp/catkin_ws \
    ./start_map_edit.sh /tmp/maps/my_map.yaml
```

If the launcher complains the workspace isn't built, rebuild it:

```bash
docker exec "$CONTAINER" bash -c "
    source /opt/ros/noetic/setup.bash && cd $CATKIN_WS && catkin_make
"
```
