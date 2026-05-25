#!/bin/bash
# Tracer test for Nav2 data pipeline.
# Run inside 3d_nav_ros2 container. Assumes /goal_pose was just published.
source /opt/ros/humble/setup.bash
source /botbrain_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7448"]'

echo "============= Tracer 1: /plan ============="
echo "--- topic info ---"
timeout 4 ros2 topic info -v /plan 2>&1 | head -16
echo "--- echo --once (Path message) ---"
timeout 6 ros2 topic echo /plan --once 2>&1 | head -25

echo
echo "============= Tracer 2: /cmd_vel_nav ============="
echo "--- topic info ---"
timeout 4 ros2 topic info /cmd_vel_nav 2>&1
echo "--- hz over 5s window ---"
timeout 6 ros2 topic hz /cmd_vel_nav 2>&1 | grep -E "average|window|no new" | head -3
echo "--- echo --once ---"
timeout 4 ros2 topic echo /cmd_vel_nav --once 2>&1 | head -8

echo
echo "============= Tracer 3: /cmd_vel ============="
echo "--- topic info ---"
timeout 4 ros2 topic info /cmd_vel 2>&1
echo "--- hz over 5s window ---"
timeout 6 ros2 topic hz /cmd_vel 2>&1 | grep -E "average|window|no new" | head -3
echo "--- echo --once ---"
timeout 4 ros2 topic echo /cmd_vel --once 2>&1 | head -8

echo
echo "============= Tracer 4: pipeline summary ============="
echo "--- recent /goal_pose ---"
timeout 3 ros2 topic echo /goal_pose --once 2>&1 | head -10

echo
echo "--- Nav2 lifecycle states (must all be active [3]) ---"
for n in controller_server planner_server bt_navigator behavior_server smoother_server waypoint_follower; do
    state=$(timeout 3 ros2 lifecycle get /$n 2>&1 | tail -1)
    echo "  $n: $state"
done

echo
echo "--- /odom hz (Nav2 needs odom!) ---"
timeout 5 ros2 topic hz /odom 2>&1 | grep -E "average|no new" | head -2
echo "--- /Odometry_loc hz (FAST-LIO native) ---"
timeout 5 ros2 topic hz /Odometry_loc 2>&1 | grep -E "average|no new" | head -2
