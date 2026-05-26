#!/bin/bash
# G1 soft stop — zero velocity, stay standing in sport mode.
#
# Cancels every in-flight goal on /navigate_to_pose. The chain that
# makes G1 actually stop:
#   1. Cancel reaches bt_navigator -> nav2 stops publishing /cmd_vel_nav
#   2. twist_mux's cmd_vel_nav input timeout (0.2s) elapses
#   3. twist_mux falls back to /cmd_vel_zero (priority 1, 0 Twist)
#      which zero_vel_publisher has been publishing since [2.5/3] of
#      nav2_launch.sh
#   4. g1_write_node receives 0 Twist, calls SDK Move(0,0,0,false)
#   5. G1 stops in place, still standing
#
# This is the "soft" brake: G1 stays in sport mode and is immediately
# ready to accept a new goal. No squat, no FSM transition, no recovery
# needed.
#
# Behavior is fire-and-forget (NOT a toggle). Calling again with no
# active goal returns goals_canceling=[] and is a no-op — safe to spam.
#
# Pre-conditions:
#   - 3d_nav_ros2 container is up
#   - nav2_launch.sh has been run (controller_server + bt_navigator
#     are active; zero_vel_publisher is publishing /cmd_vel_zero)
#
# Usage:
#   docker exec -it 3d_nav_ros2 /tmp/soft_stop.sh
#
# Compared to estop.sh:
#   estop.sh      -> fail-passive (squat). Real emergency.
#   soft_stop.sh  -> fail-active (stay standing). Routine "stop test".

source /opt/ros/humble/setup.bash
source /botbrain_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7448"]'

echo "Soft-stop: cancelling all /navigate_to_pose goals (G1 will stop in place, no squat)..."
# Empty '{}' means use default field values: uuid -> 16 zero bytes,
# stamp -> {0,0}. action_msgs/srv/CancelGoal interprets that as
# cancel-all (zero UUID + zero stamp). Don't pass `uuid: []` — the
# field is uint8[16] (fixed size) and ros2 cli rejects size-0 arrays
# with "must have a size of 16".
ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '{}'
