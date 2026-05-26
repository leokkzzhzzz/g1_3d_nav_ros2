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
import csv
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

# readline gives us tab-completion on label names.
try:
    import readline
    HAVE_READLINE = True
except ImportError:
    HAVE_READLINE = False

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


class UserAbort(Exception):
    """Raised inside send_one when the operator hits Ctrl+C while G1 is
    moving. We cancel the goal (zero-velocity stop, no FSM mode change —
    same effect as soft_stop.sh) and propagate so the main loop can exit."""


def wait_future_interruptible(fut, timeout_s, on_interrupt):
    """Like wait_future but raises UserAbort on Ctrl+C, after invoking
    on_interrupt() once for cleanup (typically a cancel_goal_async)."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if fut.done():
            return True
        try:
            time.sleep(POLL_PERIOD_S)
        except KeyboardInterrupt:
            on_interrupt()
            raise UserAbort()
    return False


def send_one(action_client, pose):
    goal = NavigateToPose.Goal()
    goal.pose = pose
    t0 = time.time()
    fut = action_client.send_goal_async(goal)
    # send_goal phase — no goal handle yet, nothing to cancel
    if not wait_future_interruptible(fut, 10.0, lambda: None):
        return False, "send_timeout", time.time() - t0
    handle = fut.result()
    if not handle or not handle.accepted:
        return False, "rejected", time.time() - t0
    res_fut = handle.get_result_async()
    # Result phase — Ctrl+C here cancels the active goal (soft brake)
    try:
        ok = wait_future_interruptible(
            res_fut, GOAL_TIMEOUT_S,
            on_interrupt=lambda: handle.cancel_goal_async())
    except UserAbort:
        # cancel_goal_async was already fired; nav2 stops publishing
        # /cmd_vel_nav and twist_mux falls back to /cmd_vel_zero (0 Twist).
        # G1 stops in place, still standing in sport mode — no squat.
        # Wait briefly for the cancel to land before returning.
        time.sleep(0.5)
        raise
    if not ok:
        handle.cancel_goal_async()
        return False, "timeout", time.time() - t0
    res = res_fut.result()
    return (res.status == GoalStatus.STATUS_SUCCEEDED,
            status_str(res.status), time.time() - t0)


def wait_future(fut, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if fut.done():
            return True
        time.sleep(POLL_PERIOD_S)
    return False


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
    ap.add_argument("--csv", default="/tmp/goto_history.csv",
                    help="append-only csv log of every visited segment")
    args = ap.parse_args()

    waypoints = load_yaml(args.waypoints)
    print(f"Loaded {len(waypoints)} waypoints from {args.waypoints}:")
    print_labels(waypoints)

    # Tab-completion on label names. Built-in commands and labels both
    # complete from a single fixed wordlist.
    if HAVE_READLINE:
        completion_words = sorted(waypoints.keys()) + ["list", "ls", "q", "q!", "quit", "exit"]

        def completer(text, state):
            opts = [w for w in completion_words if w.startswith(text)]
            return opts[state] if state < len(opts) else None

        readline.set_completer(completer)
        readline.set_completer_delims(" \t\n")
        readline.parse_and_bind("tab: complete")

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
    print("  <label>          send G1 to that waypoint (TAB to complete)")
    print("  list / ls        show all available labels")
    print("  q  / quit        exit")
    print("  q! / Ctrl+C      cancel current motion (zero-vel, no squat) + exit\n")
    print("To preempt mid-goal, hit Ctrl+C — same effect as soft_stop.sh:")
    print("  cancels the in-flight nav2 goal, twist_mux falls back to")
    print("  /cmd_vel_zero (0 Twist), G1 stops standing in sport mode.\n")

    # Append-only csv log of every segment, useful as an ad-hoc dataset
    # without having to set up rosbag. New file gets a header.
    csv_new = not os.path.exists(args.csv)
    csv_f = open(args.csv, "a", newline="")
    csv_w = csv.writer(csv_f)
    if csv_new:
        csv_w.writerow(["timestamp", "label", "goal_x", "goal_y", "goal_yaw_deg",
                        "nav2_status", "duration_s",
                        "reached_x", "reached_y", "reached_yaw_deg",
                        "xy_err_m", "yaw_err_deg"])
        csv_f.flush()

    def soft_brake_on_exit():
        """Best-effort cancel of any in-flight goal at exit. Called when
        operator types q! or hits Ctrl+C at the prompt while no goal is
        running (most common case is harmless no-op)."""
        try:
            from action_msgs.srv import CancelGoal
            cli = node.create_client(CancelGoal,
                                     "/navigate_to_pose/_action/cancel_goal")
            if cli.wait_for_service(timeout_sec=2.0):
                req = CancelGoal.Request()  # default-init = zero UUID = cancel-all
                fut = cli.call_async(req)
                wait_future(fut, 3.0)
        except Exception:
            pass

    aborted = False

    while True:
        try:
            line = input("goto> ").strip()
        except EOFError:
            print()
            break
        except KeyboardInterrupt:
            # Ctrl+C at the prompt itself — exit, no goal running so no
            # cancel needed (but call soft_brake_on_exit defensively).
            print()
            aborted = True
            break

        if not line:
            continue
        if line in ("q", "quit", "exit"):
            break
        if line == "q!":
            aborted = True
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
        rx = ry = ryaw = None
        xy_err = yaw_err_deg = None
        try:
            success, status, dur = send_one(ac, pose)
        except UserAbort:
            print("     Ctrl+C — goal cancelled (soft stop, G1 standing).")
            success, status, dur = False, "USER_CANCELED", 0.0
            aborted = True

        print(f"     nav2: {status} ({dur:.1f}s)")
        if success:
            reached = sample_pose(buf)
            if reached:
                rx, ry, ryaw = reached
                xy_err = math.hypot(rx - wp["x"], ry - wp["y"])
                yaw_err_deg = math.degrees(yaw_diff(ryaw, wp["yaw"]))
                print(f"     reached: ({rx:.3f}, {ry:.3f}, {math.degrees(ryaw):.1f}deg)"
                      f"  xy_err={xy_err:.3f}m  yaw_err={yaw_err_deg:.1f}deg")

        # Append regardless of success, so failures show up in the dataset
        csv_w.writerow([
            time.strftime("%Y-%m-%dT%H:%M:%S"), label,
            f"{wp['x']:.4f}", f"{wp['y']:.4f}", f"{math.degrees(wp['yaw']):.2f}",
            status, f"{dur:.2f}",
            f"{rx:.4f}" if rx is not None else "",
            f"{ry:.4f}" if ry is not None else "",
            f"{math.degrees(ryaw):.2f}" if ryaw is not None else "",
            f"{xy_err:.4f}" if xy_err is not None else "",
            f"{yaw_err_deg:.2f}" if yaw_err_deg is not None else "",
        ])
        csv_f.flush()
        print()

        if aborted:
            break

    if aborted:
        soft_brake_on_exit()

    csv_f.close()
    print(f"bye  (history appended to {args.csv})")
    executor.shutdown()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
