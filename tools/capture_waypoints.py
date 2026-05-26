#!/usr/bin/env python3
"""Interactive label-driven waypoint capture.

Operator drives G1 to a position (RC controller, sport mode), waits for
G1 to stand still, then types a label at the prompt. The script samples
the map->body transform at 30 Hz for 1 s and writes the mean pose into
the yaml file under that label.

Behaviour:
  - YAML stores waypoints as a dict keyed by label. Lookups are O(1) and
    a repeated label naturally overwrites the previous one.
  - On startup, if the output yaml already exists, its waypoints are
    loaded and shown. Capture is incremental — close the script, re-open,
    keep adding.
  - Every successful capture flushes the yaml to disk. Crash / Ctrl-C
    after the print line means the data is on disk.
  - Overwriting an existing label prompts for confirmation only when the
    new pose is more than 30 cm from the old one (small drift = silent
    refresh, large displacement = explicit "yes").

Built-in commands at the prompt (in addition to typing a label):
  list / ls          → print all current labels with their poses
  del <label>        → delete one label
  q / quit / exit    → save and quit (Ctrl-D works too)

Label syntax:  [A-Za-z][A-Za-z0-9_-]*   (no spaces, no special chars)

Usage:
    docker exec -it 3d_nav_ros2 bash -lc "
      source /opt/ros/humble/setup.bash
      source /botbrain_ws/install/setup.bash
      python3 /tmp/capture_waypoints.py /tmp/waypoints.yaml
    "

Requires the localization stack (launch.sh) running so map->body TF is
flowing. Does NOT require nav2_launch.sh.
"""
import argparse
import math
import os
import re
import sys
import time
from threading import Thread

# RMW must match the running stack (rmw_zenoh_cpp). See README.
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
os.environ.setdefault(
    "ZENOH_CONFIG_OVERRIDE",
    'mode="client";connect/endpoints=["tcp/127.0.0.1:7448"]',
)

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import yaml


SAMPLE_HZ = 30
SAMPLE_DURATION_S = 1.0
SOURCE_FRAME = "map"
TARGET_FRAME = "body"
TF_STREAM_WAIT_S = 10.0
OVERWRITE_PROMPT_DIST_M = 0.30
LABEL_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_-]*$")


def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def normalize_quat(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return qx / n, qy / n, qz / n, qw / n


def sample_pose(buf, duration_s):
    samples = []
    end = time.time() + duration_s
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
    z = sum(s.translation.z for s in samples) / n
    qx = sum(s.rotation.x for s in samples) / n
    qy = sum(s.rotation.y for s in samples) / n
    qz = sum(s.rotation.z for s in samples) / n
    qw = sum(s.rotation.w for s in samples) / n
    qx, qy, qz, qw = normalize_quat(qx, qy, qz, qw)
    return {
        "x": round(x, 4), "y": round(y, 4), "z": round(z, 4),
        "qx": round(qx, 6), "qy": round(qy, 6),
        "qz": round(qz, 6), "qw": round(qw, 6),
        "yaw": round(quat_to_yaw(qx, qy, qz, qw), 4),
        "samples": n,
    }


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


def load_existing(path):
    if not os.path.exists(path):
        return {}
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        wps = data.get("waypoints", {}) or {}
        if isinstance(wps, list):
            print(f"  warning: {path} is in legacy list format; converting to dict.")
            wps = {wp.get("name", f"wp{i+1}"): {k: v for k, v in wp.items() if k != "name"}
                   for i, wp in enumerate(wps)}
        return wps
    except Exception as e:
        print(f"  failed to load existing yaml: {e}")
        return {}


def save_yaml(path, waypoints):
    out = {"frame_id": SOURCE_FRAME, "waypoints": waypoints}
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        yaml.dump(out, f, sort_keys=False, default_flow_style=False)
    os.replace(tmp, path)


def print_pose_line(label, p):
    print(f"  {label:<20} x={p['x']:7.3f}  y={p['y']:7.3f}  "
          f"yaw={math.degrees(p['yaw']):6.1f}deg")


def cmd_list(waypoints):
    if not waypoints:
        print("  (no waypoints yet)")
        return
    for label in sorted(waypoints.keys()):
        print_pose_line(label, waypoints[label])
    print(f"  total: {len(waypoints)}")


def cmd_del(waypoints, label, path):
    if label not in waypoints:
        print(f"  no such label: {label!r}")
        return
    del waypoints[label]
    save_yaml(path, waypoints)
    print(f"  deleted {label}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("output", help="output yaml path (will load + append if exists)")
    args = ap.parse_args()

    rclpy.init()
    node = Node("waypoint_capture")
    buf = Buffer()
    TransformListener(buf, node)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    Thread(target=executor.spin, daemon=True).start()

    print(f"Checking {SOURCE_FRAME}->{TARGET_FRAME} TF stream...", end=" ", flush=True)
    if not wait_for_tf_stream(buf, TF_STREAM_WAIT_S):
        print(f"TIMEOUT after {TF_STREAM_WAIT_S:.0f}s")
        print("  -> launch.sh is not running, or fast_lio / open3d_loc didn't")
        print("     start. Run /root/launch.sh and wait for ALL 6 NODES RUNNING.")
        executor.shutdown(); rclpy.shutdown(); return 1
    print("OK")

    waypoints = load_existing(args.output)
    if waypoints:
        print(f"\nLoaded {len(waypoints)} existing waypoints from {args.output}:")
        cmd_list(waypoints)
    else:
        print(f"\nStarting fresh; output -> {args.output}")

    print("\nCommands at prompt:")
    print("  <label>          capture current pose under that label")
    print("  list / ls        show all captured waypoints")
    print("  del <label>      delete a waypoint")
    print("  q / quit         save and exit (Ctrl-D works too)\n")

    while True:
        try:
            line = input("wp> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        if line in ("q", "quit", "exit"):
            break
        if line in ("list", "ls"):
            cmd_list(waypoints); continue
        if line.startswith("del "):
            cmd_del(waypoints, line[4:].strip(), args.output); continue

        label = line
        if not LABEL_RE.match(label):
            print(f"  invalid label {label!r}: must match [A-Za-z][A-Za-z0-9_-]*")
            continue

        old = waypoints.get(label)
        pose = sample_pose(buf, SAMPLE_DURATION_S)
        if pose is None:
            print("  FAILED to read TF during sampling.")
            continue

        if old is not None:
            d = math.hypot(pose["x"] - old["x"], pose["y"] - old["y"])
            if d > OVERWRITE_PROMPT_DIST_M:
                ans = input(f"  {label!r} exists at ({old['x']:.2f},{old['y']:.2f}); "
                            f"new is ({pose['x']:.2f},{pose['y']:.2f}), {d:.2f}m away. "
                            f"Overwrite? (y/N): ").strip().lower()
                if ans not in ("y", "yes"):
                    print("  skipped")
                    continue
            else:
                print(f"  refreshing {label} ({d*100:.1f} cm shift)")

        waypoints[label] = pose
        save_yaml(args.output, waypoints)
        print(f"  captured ", end="")
        print_pose_line(label, pose)

    save_yaml(args.output, waypoints)
    print(f"\nSaved {len(waypoints)} waypoints to {args.output}.")
    executor.shutdown()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
