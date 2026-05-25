# Domain glossary — `3d_nav_ros2` runtime

Definitions for the recurring concepts in this stack. If a term you're hunting
isn't here, it isn't a runtime concept — check `docs/DECISIONS.md` for design
choices or skim the upstream package docs.

## ROS 2 native runtime
The set of nodes inside `3d_nav_ros2` that runs continuously on G1 to provide
localization, map, scan, and TF for navigation. All run under
`RMW_IMPLEMENTATION=rmw_zenoh_cpp` and share a single in-container Zenoh router
on `tcp/127.0.0.1:7448`.
*Avoid:* mixing ROS 1 mapping logic into runtime. Online mapping is a separate
flow (HongTu container) that produces `scans.pcd` once.

## Local localization chain
The realtime pose pipeline inside the container:
LiDAR → FAST-LIO (odometry) → open3d_loc (ICP against pre-built PCD) → TF.
*Avoid:* depending on ROS 1 containers for runtime localization. Splitting
odometry and global re-localization across different sources of truth.

## Local obstacle scan
The `/scan` LaserScan topic produced by `pointcloud_to_laserscan` from
`/cloud_registered_body_1`. The primary obstacle input for the navigation layer.
*Avoid:* using raw 3D point clouds as a hard nav dependency, which would block
closed-loop control on display-completeness.

## Cross-host visualization
The pipeline that exposes ROS 2 graph state from G1 to Leo for RViz2 monitoring,
manual `/initialpose` setting, and goal publication. Carried by the same Zenoh
router; **does not** carry control loops.

## G1 motion control
The layer that converts navigation `Twist` into Unitree SDK2 motion commands.
Currently lives in the `g1_nav_bb` container, separate from this runtime.
*Avoid:* merging control into the localization runtime.

## ICP confidence
`/localization_3d_confidence` (Float32). The published score from open3d_loc's
ICP fitness evaluation. Below the configured `confidence_loc_th` (default 0.7)
the loop does **not** publish a fresh `/localization_3d` and `map → odom` is not
corrected. Confidence stuck at 0.0 means the initial pose is wrong.

## Initial pose
The seed transform open3d_loc uses to anchor ICP. Either configured statically
in `loc_param_g1.yaml#initialpose` or set at runtime via `/initialpose` (RViz2
"2D Pose Estimate" tool). **Critical for stability** — without a correct initial
pose, fast_lio will eventually run unbounded under pure-IMU integration.

## Kalman parameter naming convention
open3d_loc declares Kalman parameters with **slashes** in the name
(`kf_baselink2map/x`), not nested YAML keys. Configs must use the slash form.
*Avoid:* writing `kf_baselink2map: { x: [...] }` — the C++ side will silently
read default zeros and the filter outputs zero correction.

## QoS — transient_local for "always-on" topics
For ROS 1 latched topics ported to ROS 2 (notably `/map` for the PCD), the
publisher must use `rclcpp::QoS(...).transient_local()`. Default ROS 2 publishers
are volatile and silently drop messages for late-joining subscribers. RViz2 is
always a late joiner.
