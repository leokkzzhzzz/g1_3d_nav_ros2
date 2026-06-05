#!/usr/bin/env python3
"""Send G1 to a single waypoint or walk a numbered series by label.

Reads a waypoints YAML file produced by capture_waypoints.py (dict format
keyed by label). Lists the available labels, then enters an interactive
loop:

  goto> office1            # single waypoint, sends one NavigateToPose
  goto> office[]           # series traversal: office1 -> office2 -> ...
                           # in numeric order, retrying each up to 3
                           # times before aborting the whole sequence
  goto> kitchen            # any non-series label still works as before

Series detection: any label matching `^([A-Za-z][A-Za-z_]*)(\d+)$` is a
member of the named group. office10 lands after office9 (sorted by
integer suffix). office_1, dock_a3 are also valid members of office_[]
and dock_a[].

Use this for ad-hoc inspection of individual waypoints — drive G1 to the
"kitchen" waypoint, look at it, drive to "door1", look at it. For
batch statistical accuracy across many waypoints in a fixed schedule,
use navigate_batch.py instead.

Usage:
    docker exec -it 3d_nav_ros2 bash -lc "
      source /opt/ros/humble/setup.bash
      source /botbrain_ws/install/setup.bash
      python3 /g1_3d_nav_ros2/tools/gotop/goto_waypoint.py /tmp/waypoints.yaml
    "

Optional flags:
    --dwell N            seconds to stand at each series waypoint after
                         arrival before firing the next goal (default 0).
                         Useful for inspection patrols, sensor reads, or
                         letting ICP settle between hops.
    --retries N          per-waypoint retry budget on ABORTED/TIMEOUT
                         (default 3). After N failures the series aborts.

While G1 is moving, the script blocks on the action result. To preempt,
run /g1_3d_nav_ros2/tools/soft_stop.sh in a separate window — it cancels
the current goal and G1 stops in place still standing in sport mode.
Inside a series, Ctrl+C cancels the current hop AND aborts the whole
sequence (returns to the prompt; later hops are not sent).

Built-in commands at the prompt:
  <label>            navigate to that waypoint
  <name>[]           walk the entire series in numeric order
  list / ls          show all available labels (series folded)
  q / quit / exit    quit (Ctrl-D works too)
"""
import argparse
import csv
import json
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
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
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
RETRY_BACKOFF_S = 1.0

# Series detection: prefix is letters/underscores only (no digits or
# punctuation in prefix), suffix is one or more digits. Sorted by integer
# value so office10 lands after office9. Mirror of capture_waypoints.py.
import re as _re
SERIES_RE = _re.compile(r"^([A-Za-z一-鿿][A-Za-z_一-鿿]*)(\d+)$")
SERIES_BRACKET_RE = _re.compile(r"^([A-Za-z一-鿿][A-Za-z_一-鿿]*)\[\]$")


def parse_series_member(label):
    """Return (group, idx_int) if label is a series member, else None."""
    m = SERIES_RE.match(label)
    if not m:
        return None
    return m.group(1), int(m.group(2))


def compute_series(waypoints):
    """Return {group: [(idx, label), ...]} sorted by idx."""
    series = {}
    for label in waypoints:
        parsed = parse_series_member(label)
        if parsed is None:
            continue
        group, idx = parsed
        series.setdefault(group, []).append((idx, label))
    for group in series:
        series[group].sort(key=lambda t: t[0])
    return series


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
    series = compute_series(waypoints)
    series_members = {label for entries in series.values() for _, label in entries}
    singles = sorted(label for label in waypoints if label not in series_members)
    for label in singles:
        wp = waypoints[label]
        print(f"  {label:<20} x={wp['x']:7.3f}  y={wp['y']:7.3f}  "
              f"yaw={math.degrees(wp['yaw']):6.1f}deg")
    for group in sorted(series.keys()):
        entries = series[group]
        idxs = [i for i, _ in entries]
        xs = [waypoints[lab]["x"] for _, lab in entries]
        ys = [waypoints[lab]["y"] for _, lab in entries]
        idx_str = (f"{group}{idxs[0]}..{group}{idxs[-1]}"
                   if len(idxs) > 1 and idxs == list(range(idxs[0], idxs[-1] + 1))
                   else ", ".join(f"{group}{i}" for i in idxs))
        print(f"  {group + '[]':<20} ({len(entries)} pts) {idx_str}  "
              f"x∈[{min(xs):.2f},{max(xs):.2f}]  y∈[{min(ys):.2f},{max(ys):.2f}]")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("waypoints", help="yaml from capture_waypoints.py")
    ap.add_argument("--csv", default="/tmp/goto_history.csv",
                    help="append-only csv log of every visited segment")
    ap.add_argument("--dwell", type=float, default=0.0,
                    help="seconds to dwell at each series waypoint after arrival "
                         "before sending the next goal (default 0)")
    ap.add_argument("--retries", type=int, default=3,
                    help="per-waypoint retry budget on ABORTED/TIMEOUT before "
                         "aborting the series (default 3)")
    args = ap.parse_args()

    waypoints = load_yaml(args.waypoints)
    print(f"Loaded {len(waypoints)} waypoints from {args.waypoints}:")
    print_labels(waypoints)

    # Tab-completion on label names. Built-in commands and labels both
    # complete from a single fixed wordlist. Series groups also complete
    # to `<group>[]`.
    if HAVE_READLINE:
        groups_for_complete = sorted(compute_series(waypoints).keys())
        completion_words = (sorted(waypoints.keys())
                            + [g + "[]" for g in groups_for_complete]
                            + ["list", "ls", "q", "q!", "quit", "exit"])

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
    reach_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    reach_pub = node.create_publisher(String, "/reach_point", reach_qos)
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
    print("  <label>          send G1 to that single waypoint (TAB completes)")
    print("  <name>[]         walk a series in numeric order (e.g. office[])")
    print("  list / ls        show all available labels (series folded)")
    print("  q  / quit        exit")
    print("  q! / Ctrl+C      cancel current motion (zero-vel, no squat) + exit\n")
    print(f"Series traversal: {args.retries} retries per waypoint, then abort.")
    if args.dwell > 0:
        print(f"Dwell: {args.dwell:.1f}s standing still at each waypoint after arrival.")
    print("To preempt mid-goal, hit Ctrl+C — same effect as soft_stop.sh:")
    print("  cancels the in-flight nav2 goal, twist_mux falls back to")
    print("  /cmd_vel_zero (0 Twist), G1 stops standing in sport mode.")
    print("Inside a series, Ctrl+C also aborts the rest of the sequence.\n")

    # Append-only csv log of every segment, useful as an ad-hoc dataset
    # without having to set up rosbag. New file gets a header.
    # Schema as of D-013 series support: adds group / seq_idx / attempt
    # columns for series-traversal analytics. Singles leave them blank.
    csv_new = not os.path.exists(args.csv)
    csv_f = open(args.csv, "a", newline="")
    csv_w = csv.writer(csv_f)
    if csv_new:
        csv_w.writerow(["timestamp", "label", "goal_x", "goal_y", "goal_yaw_deg",
                        "nav2_status", "duration_s",
                        "reached_x", "reached_y", "reached_yaw_deg",
                        "xy_err_m", "yaw_err_deg",
                        "group", "seq_idx", "attempt"])
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

    def publish_reach(name, wp_type, status, rx=None, ry=None, ryaw_deg=None,
                       xy_err=None, yaw_err_deg=None):
        msg = {"name": name, "type": wp_type, "status": status,
               "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S")}
        if status == "SUCCEEDED" and rx is not None:
            msg["position"] = {"x": round(rx, 3), "y": round(ry, 3),
                               "yaw_deg": round(ryaw_deg, 1)}
            msg["error"] = {"xy_m": round(xy_err, 3),
                            "yaw_deg": round(abs(yaw_err_deg), 1)}
        s = String()
        s.data = json.dumps(msg)
        reach_pub.publish(s)
        print(f"     /reach_point: {s.data}")

    def goto_one(label, wp, group="", seq_idx="", attempt=1):
        """Send one NavigateToPose goal, sample reached pose on success,
        append a CSV row. Returns (success, status, rx, ry, ryaw_deg, xy_err, yaw_err_deg).
        Position fields are None on failure. Raises UserAbort on Ctrl+C."""
        pose = make_pose_stamped(node, wp)
        rx = ry = ryaw = None
        xy_err = yaw_err_deg = None
        success, status, dur = send_one(ac, pose)
        print(f"     nav2: {status} ({dur:.1f}s)"
              + (f"  [attempt {attempt}/{args.retries}]" if group else ""))
        if success:
            reached = sample_pose(buf)
            if reached:
                rx, ry, ryaw = reached
                xy_err = math.hypot(rx - wp["x"], ry - wp["y"])
                yaw_err_deg = math.degrees(yaw_diff(ryaw, wp["yaw"]))
                print(f"     reached: ({rx:.3f}, {ry:.3f}, {math.degrees(ryaw):.1f}deg)"
                      f"  xy_err={xy_err:.3f}m  yaw_err={yaw_err_deg:.1f}deg")
        csv_w.writerow([
            time.strftime("%Y-%m-%dT%H:%M:%S"), label,
            f"{wp['x']:.4f}", f"{wp['y']:.4f}", f"{math.degrees(wp['yaw']):.2f}",
            status, f"{dur:.2f}",
            f"{rx:.4f}" if rx is not None else "",
            f"{ry:.4f}" if ry is not None else "",
            f"{math.degrees(ryaw):.2f}" if ryaw is not None else "",
            f"{xy_err:.4f}" if xy_err is not None else "",
            f"{yaw_err_deg:.2f}" if yaw_err_deg is not None else "",
            group, str(seq_idx) if seq_idx != "" else "", str(attempt),
        ])
        csv_f.flush()
        ryaw_deg_out = math.degrees(ryaw) if ryaw is not None else None
        return success, status, rx, ry, ryaw_deg_out, xy_err, yaw_err_deg

    def walk_series(group, members):
        """members is a list of (idx, label). Walks them in order. Each
        waypoint gets up to args.retries attempts. First all-failure
        aborts the whole sequence. Ctrl+C cancels current hop and aborts.
        Returns True if completed, False if aborted (any reason)."""
        total = len(members)
        print(f"\n  walking {group}[] ({total} pts: "
              f"{', '.join(lab for _, lab in members)})")
        last_rx = last_ry = last_ryaw_deg = last_xy_err = last_yaw_err = None
        last_status = None
        for hop, (idx, label) in enumerate(members, 1):
            wp = waypoints[label]
            print(f"\n  [{hop}/{total}] -> {label} (x={wp['x']:.2f} y={wp['y']:.2f} "
                  f"yaw={math.degrees(wp['yaw']):.0f}deg)")
            success = False
            for attempt in range(1, args.retries + 1):
                if attempt > 1:
                    print(f"     retry {attempt}/{args.retries} after {RETRY_BACKOFF_S}s ...")
                    time.sleep(RETRY_BACKOFF_S)
                try:
                    success, status, rx, ry, ryaw_deg, xy_err, yaw_err_deg = \
                        goto_one(label, wp, group, idx, attempt)
                except UserAbort:
                    print("     Ctrl+C — goal cancelled. Series aborted.")
                    csv_w.writerow([
                        time.strftime("%Y-%m-%dT%H:%M:%S"), label,
                        f"{wp['x']:.4f}", f"{wp['y']:.4f}",
                        f"{math.degrees(wp['yaw']):.2f}",
                        "USER_CANCELED", "0.00", "", "", "", "", "",
                        group, str(idx), str(attempt),
                    ])
                    csv_f.flush()
                    publish_reach(group, "series", "USER_CANCELED")
                    return False
                if success:
                    last_rx, last_ry, last_ryaw_deg = rx, ry, ryaw_deg
                    last_xy_err, last_yaw_err = xy_err, yaw_err_deg
                    last_status = status
                    break
                last_status = status
            if not success:
                print(f"\n  series {group}[] aborted: "
                      f"{label} failed after {args.retries} attempts.")
                publish_reach(group, "series", last_status)
                return False
            if args.dwell > 0 and hop < total:
                print(f"     dwell {args.dwell:.1f}s ...")
                try:
                    time.sleep(args.dwell)
                except KeyboardInterrupt:
                    print("     Ctrl+C during dwell — series aborted.")
                    publish_reach(group, "series", "USER_CANCELED")
                    return False
        print(f"\n  series {group}[] completed: {total}/{total} ok.")
        publish_reach(group, "series", "SUCCEEDED",
                      last_rx, last_ry, last_ryaw_deg, last_xy_err, last_yaw_err)
        return True

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

        # Series traversal: `<name>[]`
        bm = SERIES_BRACKET_RE.match(line)
        if bm:
            group = bm.group(1)
            current_series = compute_series(waypoints)
            if group not in current_series:
                print(f"  no series {group}[] in this yaml. "
                      f"`list` shows what's available.")
                continue
            ok = walk_series(group, current_series[group])
            if not ok:
                aborted = True
                break
            print()
            continue

        label = line
        if label not in waypoints:
            # Helpful: if user typed a bare group name, suggest the [] form
            current_series = compute_series(waypoints)
            if label in current_series:
                print(f"  {label!r} is a series, not a single point. "
                      f"Use {label}[] to walk it (or {label}1 to test member 1).")
            else:
                print(f"  no such label: {label!r}. Type 'list' to see options.")
            continue

        wp = waypoints[label]
        print(f"\n  -> {label} (x={wp['x']:.2f} y={wp['y']:.2f} "
              f"yaw={math.degrees(wp['yaw']):.0f}deg)")
        try:
            success, status, rx, ry, ryaw_deg, xy_err, yaw_err_deg = goto_one(label, wp)
            publish_reach(label, "single", status, rx, ry, ryaw_deg, xy_err, yaw_err_deg)
        except UserAbort:
            print("     Ctrl+C — goal cancelled (soft stop, G1 standing).")
            csv_w.writerow([
                time.strftime("%Y-%m-%dT%H:%M:%S"), label,
                f"{wp['x']:.4f}", f"{wp['y']:.4f}",
                f"{math.degrees(wp['yaw']):.2f}",
                "USER_CANCELED", "0.00", "", "", "", "", "",
                "", "", "1",
            ])
            csv_f.flush()
            publish_reach(label, "single", "USER_CANCELED")
            aborted = True
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
