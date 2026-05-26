#!/bin/bash
# G1 emergency stop — fail-passive brake (zero velocity + squat down).
#
# Calls /emergency_stop service on g1_write_node, which:
#   1. sets emergency_flag_ -> cmd_vel callback drops all incoming Twists
#   2. calls SDK stop_move() (zero velocity)
#   3. ~1s later, sets FSM to BALANCE_SQUAT_SQUAT_STAND (G1 squats)
#
# Behavior is a TOGGLE: first call = ON (stop+squat), second call = OFF
# (G1 stands up + cmd_vel callback resumes accepting Twists).
#
# When to use: real emergency. G1 about to fall, run into something
# heavy, walk off a ledge. Squat is fail-passive — even if balance
# fails, G1 ends up in a low stable posture.
#
# When NOT to use: routine "stop the test" — use soft_stop.sh instead,
# which leaves G1 standing in sport mode (faster to resume).
#
# Pre-conditions:
#   - 3d_nav_ros2 container is up
#   - launch.sh + nav2_launch.sh have been run (g1_write_node is active)
#
# Usage:
#   docker exec -it 3d_nav_ros2 /tmp/estop.sh

source /opt/ros/humble/setup.bash
source /botbrain_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/127.0.0.1:7448"]'

echo "Calling /emergency_stop (toggle: 1st = ON+squat, 2nd = OFF)..."
ros2 service call /emergency_stop std_srvs/srv/SetBool "{}"
