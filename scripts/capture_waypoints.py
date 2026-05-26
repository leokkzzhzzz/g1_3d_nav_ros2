#!/usr/bin/env python3
"""Capture N waypoints by reading map->body TF, averaged over 1s.

Operator drives G1 to each waypoint (RC controller, sport mode), waits for
G1 to be physically stationary, then presses Enter. The script samples the
map->body transform at 30 Hz for 1 s, takes the mean (xyz + quaternion),
and writes one waypoint entry to a YAML file.

Usage:
    docker exec -it 3d_nav_ros2 bash -lc "
      source /opt/ros/humble/setup.bash
      source /botbrain_ws/install/setup.bash
      python3 /tmp/capture_waypoints.py /tmp/waypoints.yaml --count 5
    "

Requires the localization stack (launch.sh) running so map->body TF is
flowing. Does NOT require nav2_launch.sh.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import yaml


SAMPLE_HZ = 30
SAMPLE_DURATION_S = 1.0
SOURCE_FRAME = "map"
TARGET_FRAME = "body"


def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def normalize_quat(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return qx / n, qy / n, qz / n, qw / n


def sample_pose(node, buf, duration_s):
    samples = []
    end = time.time() + duration_s
    period = 1.0 / SAMPLE_HZ
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=period)
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("output", help="output yaml path")
    ap.add_argument("--count", type=int, default=5)
    args = ap.parse_args()

    rclpy.init()
    node = Node("waypoint_capture")
    buf = Buffer()
    TransformListener(buf, node)

    print(f"Will capture {args.count} waypoints; output -> {args.output}")
    print(f"For each waypoint: drive G1 there, wait until physically still, press Enter.\n")

    waypoints = []
    for i in range(1, args.count + 1):
        input(f"  waypoint {i}/{args.count} — press Enter when G1 is stable: ")
        pose = sample_pose(node, buf, SAMPLE_DURATION_S)
        if pose is None:
            print(f"    FAILED to read TF (is launch.sh running?). Aborting.")
            return 1
        pose["name"] = f"wp{i}"
        print(f"    captured wp{i}: x={pose['x']:.3f} y={pose['y']:.3f} "
              f"yaw={math.degrees(pose['yaw']):.1f}deg ({pose['samples']} samples)")
        waypoints.append(pose)

    out = {
        "frame_id": SOURCE_FRAME,
        "waypoints": waypoints,
    }
    with open(args.output, "w") as f:
        yaml.dump(out, f, sort_keys=False, default_flow_style=False)
    print(f"\nWrote {len(waypoints)} waypoints to {args.output}.")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
