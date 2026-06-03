# G1 RGB Navigation

This repository now carries two separate G1 navigation projects. `3d_nav_ros2` is the existing FAST_LIO-based stack, and `3d_nav_rgb` is the new FAST-LIVO2-based RGB stack.

## Language

**3d_nav_ros2**:
The existing LiDAR-first G1 navigation project based on **FAST_LIO**.
_Avoid_: RGB project, new stack

**3d_nav_rgb**:
A separate G1 RGB navigation project with its own container, paths, maps, and launch scripts.
_Avoid_: patch mode, compatibility mode

**FAST_LIO**:
The LiDAR-inertial front-end used by **3d_nav_ros2**.
_Avoid_: FAST-LIVO2, RGB front-end

**FAST-LIVO2**:
The LiDAR-visual-inertial front-end selected for **3d_nav_rgb**.
_Avoid_: FAST_LIO

**RGB Map**:
A single colored point-cloud map file generated for **3d_nav_rgb** at `maps/scans.pcd`.
_Avoid_: legacy map, split PCD set

**ColoredICP**:
The Open3D localization mode that must consume both a colored runtime scan and a colored **RGB Map**.
_Avoid_: geometric ICP, default ICP

## Relationships

- **3d_nav_ros2** and **3d_nav_rgb** are separate projects and do not share runtime paths or containers
- **3d_nav_ros2** uses **FAST_LIO**
- **3d_nav_rgb** uses **FAST-LIVO2**
- **FAST-LIVO2** must generate the live colored scan and the **RGB Map** required by **ColoredICP**

## Example dialogue

> **Dev:** "Can we keep the old `3d_nav_ros2` paths inside the RGB container just for convenience?"
> **Domain expert:** "No. **3d_nav_rgb** is a separate project, so it gets its own paths, maps, logs, and launch scripts."

## Flagged ambiguities

- "`rgb_version`" was used to mean both the remote git branch and the deployed RGB project. Resolved: `rgb_version` is the source branch; **3d_nav_rgb** is the deployed project.
- "`ColoredICP enabled`" was used as if changing one YAML field were enough. Resolved: **ColoredICP** is only considered enabled when the runtime scan path, map path, and localization code path all use the colored pipeline.
