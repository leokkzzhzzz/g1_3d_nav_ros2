# `/map` publisher QoS — TRANSIENT_LOCAL

Source: `src/deepglint/open3d_loc/src/global_localization.cpp`
Applied: 2026-05-25, baked into `g1_nav_final:latest` SHA `183e0426c630...`

## Patch

Around line 268, change:

```cpp
pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map", 1);
```

To:

```cpp
pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/map",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
```

## Why

ROS 2 default publisher QoS is `RELIABLE` + `VOLATILE`. With `VOLATILE` the
publisher does not buffer messages for subscribers that connect after a
message was sent. open3d_loc publishes `/map` once at startup — RViz2 launches
later, subscribes, never sees it. Symptom in RViz2: PointCloud2 display shows
`Showing [0] points from [0] messages`, status OK, publisher visible.

ROS 1 latched topics behave like ROS 2 `transient_local`. Porting to ROS 2
without changing the QoS silently breaks late-joining subscribers.

## Apply

```bash
docker exec 3d_nav_ros2 bash -lc '
  cd /root/3d_nav_g1/g1_ws
  source /opt/ros/humble/setup.bash
  colcon build --packages-select open3d_loc --symlink-install
'
docker commit 3d_nav_ros2 g1_nav_final:latest
```

## Future work

Other topics in `global_localization.cpp` (lines 268-275) currently use
`create_publisher<T>("...", 1)` (volatile). If any of `/submap`, `/scan2map`,
`/baselink2map`, `/odom2map`, `/odom2map_kalman` need to reach late-joining
RViz2 / Nav2 sessions, apply the same QoS transformation.
