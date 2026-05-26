#!/usr/bin/env python3
"""Send G1 to a single waypoint by label.

Reads a waypoints YAML file produced by capture_waypoints.py (dict format
keyed by label). Lists the available labels, then enters an interactive
loop: type a label, the script sends a NavigateToPose goal there, waits
for the result, reports the achieved-vs-goal pose error.

Use this for ad-hoc inspection of individual waypoints — drive G1 to the
"kitchen" waypoint, look at it, drive to "door1", look at it. For
batch statistical accuracy across many waypoints in a fixed schedule,
use navigate_batch.py instead.

Usage:
    docker exec -it 3d_nav_ros2 bash -lc "
      source /opt/ros/humble/setup.bash
      source /botbrain_ws/install/setup.bash
      python3 /tmp/goto_waypoint.py /tmp/waypoints.yaml
    "

While G1 is moving, the script blocks on the action result. To preempt,
run /tmp/soft_stop.sh in a separate window — it cancels the current goal
and G1 stops in place still standing in sport mode.

Built-in commands at the prompt:
  <label>            navigate to that waypoint
  list / ls          show all available labels
  q / quit / exit    quit (Ctrl-D works too)
"""
import argparse
import math
import os
import sys
import time
from threading import Thread

# Set RMW env before any rclpy / DDS C-extension import.
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
os.environ.setdefault(
    "ZENOH_CONFIG_OVERRIDE",
    'mode="client";connect/endpoints=["tcp/127.0.0.1:7448"]',
)

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import yaml


SAMPLE_HZ = 30
SAMPLE_DURATION_S = 1.0
SOURCE_FRAME = "map"
TARGET_FRAME = "body"
GOAL_TIMEOUT_S = 120.0
TF_STREAM_WAIT_S = 10.0
ACTION_SERVER_WAIT_S = 15.0
POLL_PERIOD_S = 0.05


def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def yaw_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def sample_pose(buf):
    samples = []
    end = time.time() + SAMPLE_DURATION_S
    period = 1.0 / SAMPLE_HZ
    while time.time() < end:
        try:
            t = buf.lookup_transform(SOURCE_FRAME, TARGET_FRAME,
                                     rclpy.time.Time())
            samples.append(t.transform)
        except Exception:
            pass
        time.sleep(period)
    if not samples:
        return None
    n = len(samples)
    x = sum(s.translation.x for s in samples) / n
    y = sum(s.translation.y for s in samples) / n
    qx = sum(s.rotation.x for s in samples) / n
    qy = sum(s.rotation.y for s in samples) / n
    qz = sum(s.rotation.z for s in samples) / n
    qw = sum(s.rotation.w for s in samples) / n
    return x, y, quat_to_yaw(qx, qy, qz, qw)


def make_pose_stamped(node, wp):
    p = PoseStamped()
    p.header.frame_id = SOURCE_FRAME
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x = float(wp["x"])
    p.pose.position.y = float(wp["y"])
    p.pose.position.z = 0.0
    p.pose.orientation.x = float(wp["qx"])
    p.pose.orientation.y = float(wp["qy"])
    p.pose.orientation.z = float(wp["qz"])
    p.pose.orientation.w = float(wp["qw"])
    return p


def status_str(s):
    return {
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
    }.get(s, f"STATUS_{s}")


def wait_future(fut, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if fut.done():
            return True
        time.sleep(POLL_PERIOD_S)
    return False


def send_one(action_client, pose):
    goal = NavigateToPose.Goal()
    goal.pose = pose
    t0 = time.time()
    fut = action_client.send_goal_async(goal)
    if not wait_future(fut, 10.0):
        return False, "send_timeout", time.time() - t0
    handle = fut.result()
    if not handle or not handle.accepted:
        return False, "rejected", time.time() - t0
    res_fut = handle.get_result_async()
    if not wait_future(res_fut, GOAL_TIMEOUT_S):
        handle.cancel_goal_async()
        return False, "timeout", time.time() - t0
    res = res_fut.result()
    return (res.status == GoalStatus.STATUS_SUCCEEDED,
            status_str(res.status), time.time() - t0)


def wait_for_tf_stream(buf, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            buf.lookup_transform(SOURCE_FRAME, TARGET_FRAME,
                                 rclpy.time.Time())
            return True
        except Exception:
            time.sleep(0.2)
    return False


def load_yaml(path):
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    wps = data.get("waypoints", {}) or {}
    if isinstance(wps, list):
        wps = {wp.get("name", f"wp{i+1}"): {k: v for k, v in wp.items() if k != "name"}
               for i, wp in enumerate(wps)}
    return wps


def print_labels(waypoints):
    if not waypoints:
        print("  (no waypoints in file)")
        return
    for label in sorted(waypoints.keys()):
        wp = waypoints[label]
        print(f"  {label:<20} x={wp['x']:7.3f}  y={wp['y']:7.3f}  "
              f"yaw={math.degrees(wp['yaw']):6.1f}deg")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("waypoints", help="yaml from capture_waypoints.py")
    args = ap.parse_args()

    waypoints = load_yaml(args.waypoints)
    print(f"Loaded {len(waypoints)} waypoints from {args.waypoints}:")
    print_labels(waypoints)

    rclpy.init()
    node = Node("goto_waypoint")
    buf = Buffer()
    TransformListener(buf, node)
    ac = ActionClient(node, NavigateToPose, "navigate_to_pose")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    Thread(target=executor.spin, daemon=True).start()

    print(f"\nChecking {SOURCE_FRAME}->{TARGET_FRAME} TF stream...", end=" ", flush=True)
    if not wait_for_tf_stream(buf, TF_STREAM_WAIT_S):
        print("TIMEOUT — is launch.sh running?")
        executor.shutdown(); rclpy.shutdown(); return 1
    print("OK")

    print("Waiting for /navigate_to_pose action server...", end=" ", flush=True)
    if not ac.wait_for_server(timeout_sec=ACTION_SERVER_WAIT_S):
        print("TIMEOUT — is nav2_launch.sh running?")
        executor.shutdown(); rclpy.shutdown(); return 1
    print("OK")

    print("\nCommands at prompt:")
    print("  <label>          send G1 to that waypoint")
    print("  list / ls        show all available labels")
    print("  q / quit         exit\n")
    print("To preempt mid-goal, run /tmp/soft_stop.sh from another window.\n")

    while True:
        try:
            line = input("goto> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        if line in ("q", "quit", "exit"):
            break
        if line in ("list", "ls"):
            print_labels(waypoints); continue

        label = line
        if label not in waypoints:
            print(f"  no such label: {label!r}. Type 'list' to see options.")
            continue

        wp = waypoints[label]
        print(f"\n  -> {label} (x={wp['x']:.2f} y={wp['y']:.2f} "
              f"yaw={math.degrees(wp['yaw']):.0f}deg)")
        pose = make_pose_stamped(node, wp)
        success, status, dur = send_one(ac, pose)
        print(f"     nav2: {status} ({dur:.1f}s)")
        if success:
            reached = sample_pose(buf)
            if reached:
                rx, ry, ryaw = reached
                xy_err = math.hypot(rx - wp["x"], ry - wp["y"])
                yaw_err_deg = math.degrees(yaw_diff(ryaw, wp["yaw"]))
                print(f"     reached: ({rx:.3f}, {ry:.3f}, {math.degrees(ryaw):.1f}deg)"
                      f"  xy_err={xy_err:.3f}m  yaw_err={yaw_err_deg:.1f}deg")
        print()

    print("bye")
    executor.shutdown()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
