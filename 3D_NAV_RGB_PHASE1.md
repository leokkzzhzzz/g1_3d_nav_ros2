# 3D Nav RGB Phase 1

## Goal

Build the first runnable `3d_nav_rgb` project as a separate stack from `3d_nav_ros2`, using `FAST-LIVO2` for mapping and runtime localization input, and using `ColoredICP` in `open3d_loc`.

## Confirmed Decisions

- The RGB project is deployed at `/home/unitree/g1_3d_nav_rgb`.
- The RGB container name is `3d_nav_rgb`.
- The RGB container-internal repo path is `/g1_3d_nav_rgb`.
- The RGB project is initialized by cloning `origin/rgb_version` from GitHub.
- `FAST-LIVO2` is pulled from a single upstream remote source, not copied from another local workspace.
- `3d_nav_rgb` is a separate project, not a compatibility layer over `3d_nav_ros2`.
- Phase 1 includes both RGB mapping and RGB runtime localization input.
- Phase 1 uses independent RGB scripts rather than modifying the old launch scripts in place.
- `open3d_loc` is allowed to adapt to the new FAST-LIVO2 runtime interface.
- Phase 1 keeps Nav2 and `pointcloud_to_laserscan` out of scope.
- The RGB map default path is `/g1_3d_nav_rgb/maps/scans.pcd`.
- Mapping stop must automatically produce a single colored `maps/scans.pcd`.
- RGB logs are separate from the old project logs.

## Phase 1 Scope

### 1. Create the RGB project skeleton

- Clone `origin/rgb_version` to `/home/unitree/g1_3d_nav_rgb`
- Create a new runtime container script for `3d_nav_rgb`
- Use RGB-specific mount points, paths, and log names

### 2. Introduce FAST-LIVO2

- Remove `FAST_LIO` from the RGB project workspace
- Add `FAST-LIVO2` under:
  `3d_nav_g1/g1_ws/src/deepglint/FAST_LIVO2`
- Add RGB-specific launch scripts rather than reusing the old ones

### 3. Build the RGB mapping path

- Add an RGB mapping launch script
- Add an RGB mapping save script
- Make mapping stop produce:
  `/g1_3d_nav_rgb/maps/scans.pcd`
- Require the saved map to satisfy:
  `has_colors() == True`

### 4. Enable ColoredICP in open3d_loc

- Add `icp_method == 3` in `open3d_registration.cpp`
- Update the `open3d_registration.h` method documentation
- Set `icp_method: 3` in the RGB localization config
- Change `global_localization.cpp` so it actually uses configured `icp_method`
- Point the RGB localization launch path at:
  `/g1_3d_nav_rgb/maps/scans.pcd`

### 5. Retarget runtime localization inputs

- Move `open3d_loc` off the old FAST_LIO runtime assumptions
- Retarget it to the FAST-LIVO2 runtime outputs selected for the RGB stack
- Validate the runtime chain:
  `FAST-LIVO2 -> colored live scan -> open3d_loc(ColoredICP)`

## Out of Scope for Phase 1

- `pointcloud_to_laserscan`
- `nav2_launch.sh`
- `gotop batch`
- Any compatibility layer intended to keep old `3d_nav_ros2` runtime interfaces alive inside `3d_nav_rgb`

## Files Expected to Change

- `3d_nav_g1/g1_ws/src/deepglint/open3d_loc/src/open3d_registration/open3d_registration.cpp`
- `3d_nav_g1/g1_ws/src/deepglint/open3d_loc/include/open3d_registration/open3d_registration.h`
- `3d_nav_g1/g1_ws/src/deepglint/open3d_loc/src/global_localization.cpp`
- `3d_nav_g1/g1_ws/src/deepglint/open3d_loc/config/loc_param_g1.yaml`
- `3d_nav_g1/g1_ws/src/deepglint/open3d_loc/launch/open3d_loc_g1.launch.py`
- `tools/recreate_3d_nav_rgb.sh`
- `tools/mapping/mapping_launch_rgb.sh`
- `tools/mapping/mapping_save_rgb.sh`
- `tools/nav/launch_rgb.sh`

## Acceptance Criteria

- `3d_nav_rgb` can start in its own container without using the old project paths
- `FAST-LIVO2` starts inside the RGB project and produces runtime outputs for localization
- RGB mapping produces one file at `/g1_3d_nav_rgb/maps/scans.pcd`
- That file is a colored point cloud
- `open3d_loc` runs with `ColoredICP`
- `open3d_loc` consumes the RGB project map path rather than the old map path

## Validation Commands

```bash
python3 -c "
import open3d as o3d
pcd = o3d.io.read_point_cloud('/home/unitree/g1_3d_nav_rgb/maps/scans.pcd')
print('Has colors:', pcd.has_colors())
print('Points:', len(pcd.points))
"
```

Expected:

```text
Has colors: True
```

## Open Inputs Before Execution

- The exact upstream git remote to use for `FAST-LIVO2`
- The exact runtime topic and TF contract chosen for the RGB localization chain
- Whether z-shift is executed immediately after map save in Phase 1, or as the first post-save step before localization
