#!/usr/bin/env python3
"""Batch waypoint accuracy test — drive G1 through a chosen subset of
waypoints, R rounds each, and produce a per-segment + per-waypoint
accuracy report.

Reads a waypoints YAML produced by capture_waypoints.py (dict format
keyed by label). Operator selects which labels to traverse via
`--labels a,b,c` or `--all` to traverse every label. Each round walks
the labels in the given order; default 3 rounds for stochastic
statistics.

For each segment the script:
  - sends a NavigateToPose goal to that waypoint
  - waits for the action result (max GOAL_TIMEOUT_S, default 120 s)
  - samples the achieved pose with a 1 s mean of map->body TF
  - prompts the operator for a `physical_sanity (y/n/skip)` check
  - records nav2 status, xy_err, yaw_err, duration

NavigateToPose is used in series rather than FollowWaypoints because
FollowWaypoints does not stop between waypoints, which prevents
per-segment TF measurement.

Usage:
    docker exec -it 3d_nav_ros2 bash -lc "
      source /opt/ros/humble/setup.bash
      source /botbrain_ws/install/setup.bash
      python3 /g1_3d_nav_ros2/tools/gotop/navigate_batch.py /tmp/waypoints.yaml \\
          --labels kitchen,door1,corner --rounds 3 \\
          --output /tmp/batch_report.md
    "
    # or all labels in dict-iteration order:
    #   python3 /g1_3d_nav_ros2/tools/gotop/navigate_batch.py /tmp/waypoints.yaml --all --rounds 3

Requires the full Nav2 + g1_write_node stack (nav2_launch.sh) and an
operator on site to satisfy D-011 safety preconditions and answer the
'physical_sanity' prompt after each segment.
"""
import argparse
import math
import os
import random
import sys
import time
from dataclasses import dataclass
from threading import Thread
from typing import List, Optional

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


@dataclass
class SegmentResult:
    round_idx: int
    label: str
    goal_x: float
    goal_y: float
    goal_yaw: float
    success: bool
    nav2_status: str
    reached_x: Optional[float]
    reached_y: Optional[float]
    reached_yaw: Optional[float]
    xy_err: Optional[float]
    yaw_err_deg: Optional[float]
    duration_s: float
    physical_sanity: str


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


def send_one(node, action_client, pose):
    if not action_client.wait_for_server(timeout_sec=ACTION_SERVER_WAIT_S):
        return False, "no_server", 0.0
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
    """Returns (waypoints_dict, groups_dict)."""
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    wps = data.get("waypoints", {}) or {}
    groups = data.get("groups", {}) or {}
    if isinstance(wps, list):
        wps = {wp.get("name", f"wp{i+1}"): {k: v for k, v in wp.items() if k != "name"}
               for i, wp in enumerate(wps)}
    return wps, groups


def write_report(results: List[SegmentResult], path):
    lines = [
        "# Batch waypoint accuracy report",
        "",
        "| Round | Label | Goal (x,y,yaw°) | Reached (x,y,yaw°) | xy_err (m) | yaw_err (°) | duration (s) | nav2 | sanity |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    for r in results:
        goal = f"({r.goal_x:.2f},{r.goal_y:.2f},{math.degrees(r.goal_yaw):.0f})"
        if r.reached_x is None:
            reached = "—"; xy = "—"; yaw = "—"
        else:
            reached = f"({r.reached_x:.2f},{r.reached_y:.2f},{math.degrees(r.reached_yaw):.0f})"
            xy = f"{r.xy_err:.3f}"
            yaw = f"{r.yaw_err_deg:.1f}"
        lines.append(f"| {r.round_idx} | {r.label} | {goal} | {reached} | "
                     f"{xy} | {yaw} | {r.duration_s:.1f} | {r.nav2_status} | {r.physical_sanity} |")
    lines.append("")
    lines.append("## Per-waypoint summary (across rounds)")
    lines.append("")
    lines.append("| Label | success_rate | mean xy_err (m) | std xy_err | mean yaw_err (°) | std yaw_err |")
    lines.append("|---|---|---|---|---|---|")
    by_label = {}
    for r in results:
        by_label.setdefault(r.label, []).append(r)
    for label in sorted(by_label):
        rs = by_label[label]
        succ = sum(1 for x in rs if x.success)
        xy = [x.xy_err for x in rs if x.xy_err is not None]
        yaw = [x.yaw_err_deg for x in rs if x.yaw_err_deg is not None]
        m_xy = sum(xy) / len(xy) if xy else float("nan")
        s_xy = (sum((v - m_xy) ** 2 for v in xy) / len(xy)) ** 0.5 if xy else float("nan")
        m_yaw = sum(yaw) / len(yaw) if yaw else float("nan")
        s_yaw = (sum((v - m_yaw) ** 2 for v in yaw) / len(yaw)) ** 0.5 if yaw else float("nan")
        lines.append(f"| {label} | {succ}/{len(rs)} | {m_xy:.3f} | {s_xy:.3f} | "
                     f"{m_yaw:.1f} | {s_yaw:.1f} |")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def parse_label_list(args, available, groups):
    """Resolve --labels into a concrete list of labels.

    --all takes precedence and returns every label in the yaml.
    Otherwise --labels is a comma-separated mix of:
      - bare labels  (must exist in `available`)
      - @groupname   (expanded to that group's member list)
    Duplicates are preserved on first occurrence and dropped on later
    repeats — `a,a,b` → `a,b`. Unknown labels / groups → empty + error.
    """
    if args.all:
        return list(available.keys())
    if not args.labels:
        return []
    raw = [s.strip() for s in args.labels.split(",") if s.strip()]
    expanded = []
    for tok in raw:
        if tok.startswith("@"):
            gname = tok[1:]
            if gname not in groups:
                print(f"  unknown group: @{gname}. Available groups: "
                      f"{', '.join(groups.keys()) or '(none)'}")
                return []
            expanded.extend(groups[gname])
        else:
            expanded.append(tok)
    # Validate + dedup (preserve first-seen order)
    seen = set()
    final = []
    missing = []
    for lab in expanded:
        if lab in seen:
            continue
        seen.add(lab)
        if lab not in available:
            missing.append(lab)
            continue
        final.append(lab)
    if missing:
        print(f"  unknown label(s): {missing}")
        return []
    return final


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("waypoints", help="yaml from capture_waypoints.py")
    ap.add_argument("--labels", help="comma-separated mix of labels and @groups (e.g. a,b,@kitchen_zone)")
    ap.add_argument("--all", action="store_true", help="visit every label in the yaml")
    ap.add_argument("--rounds", type=int, default=3)
    ap.add_argument("--shuffle", action="store_true",
                    help="randomise waypoint order within each round (different seed per round)")
    ap.add_argument("--output", default="/tmp/batch_report.md")
    args = ap.parse_args()

    if not args.labels and not args.all:
        ap.error("specify either --labels a,b,@group or --all")

    waypoints, groups = load_yaml(args.waypoints)
    labels = parse_label_list(args, waypoints, groups)
    if not labels:
        return 1

    print(f"Loaded {len(waypoints)} waypoints from {args.waypoints}")
    if groups:
        print(f"Groups defined: {', '.join(groups.keys())}")
    shuffle_note = " (shuffled per round)" if args.shuffle else ""
    print(f"Will visit {len(labels)} labels x {args.rounds} rounds = "
          f"{len(labels) * args.rounds} segments{shuffle_note}:")
    for label in labels:
        wp = waypoints[label]
        print(f"  {label:<20} x={wp['x']:7.3f}  y={wp['y']:7.3f}  "
              f"yaw={math.degrees(wp['yaw']):6.1f}deg")
    print()

    rclpy.init()
    node = Node("navigate_batch")
    buf = Buffer()
    TransformListener(buf, node)
    ac = ActionClient(node, NavigateToPose, "navigate_to_pose")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    Thread(target=executor.spin, daemon=True).start()

    print(f"Checking {SOURCE_FRAME}->{TARGET_FRAME} TF stream...", end=" ", flush=True)
    if not wait_for_tf_stream(buf, TF_STREAM_WAIT_S):
        print("TIMEOUT — is launch.sh running?")
        executor.shutdown(); rclpy.shutdown(); return 1
    print("OK")

    print("Waiting for /navigate_to_pose action server...", end=" ", flush=True)
    if not ac.wait_for_server(timeout_sec=ACTION_SERVER_WAIT_S):
        print("TIMEOUT — is nav2_launch.sh running?")
        executor.shutdown(); rclpy.shutdown(); return 1
    print("OK\n")

    results: List[SegmentResult] = []
    for round_idx in range(1, args.rounds + 1):
        round_labels = list(labels)
        if args.shuffle:
            random.shuffle(round_labels)
            print(f"\n=== Round {round_idx}/{args.rounds} (shuffled: "
                  f"{','.join(round_labels)}) ===")
        else:
            print(f"\n=== Round {round_idx}/{args.rounds} ===")
        for label in round_labels:
            wp = waypoints[label]
            print(f"\n  -> {label} (x={wp['x']:.2f} y={wp['y']:.2f} "
                  f"yaw={math.degrees(wp['yaw']):.0f}deg)")
            pose = make_pose_stamped(node, wp)
            success, status, dur = send_one(node, ac, pose)
            print(f"     nav2: {status} ({dur:.1f}s)")

            reached = sample_pose(buf) if success else None
            if reached:
                rx, ry, ryaw = reached
                xy_err = math.hypot(rx - wp["x"], ry - wp["y"])
                yaw_err_deg = math.degrees(yaw_diff(ryaw, wp["yaw"]))
                print(f"     reached: ({rx:.3f}, {ry:.3f}, {math.degrees(ryaw):.1f}deg)"
                      f"  xy_err={xy_err:.3f}m  yaw_err={yaw_err_deg:.1f}deg")
            else:
                rx = ry = ryaw = None
                xy_err = yaw_err_deg = None

            try:
                sanity = input("     physical_sanity (y/n/skip): ").strip().lower() or "skip"
            except (EOFError, KeyboardInterrupt):
                sanity = "skip"
                print()
            results.append(SegmentResult(
                round_idx=round_idx, label=label,
                goal_x=wp["x"], goal_y=wp["y"], goal_yaw=wp["yaw"],
                success=success, nav2_status=status,
                reached_x=rx, reached_y=ry, reached_yaw=ryaw,
                xy_err=xy_err, yaw_err_deg=yaw_err_deg,
                duration_s=dur, physical_sanity=sanity,
            ))

    write_report(results, args.output)
    print(f"\nReport written: {args.output}")
    executor.shutdown()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
