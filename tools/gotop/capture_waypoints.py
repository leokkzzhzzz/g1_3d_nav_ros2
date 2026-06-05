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
  del <name>[]       → delete all members of a series
  rename <old> <new> rename a waypoint (group refs updated too)
  q / quit / exit    → save and quit (Ctrl-D works too)

Label syntax:  [A-Za-z][A-Za-z0-9_-]*   (no spaces, no special chars)

Series capture (`office[]`):
  Type `<name>[]` at the prompt to capture a numbered sequence. The
  script prompts `<name>[N]>` for each member; N starts at the next free
  integer (existing office1..office4 → next is office5). Type `q` inside
  a series prompt to return to `wp>` without quitting the script.
  Series prefix must match `[A-Za-z][A-Za-z_]*` (letters/underscores
  only; no digits, hyphens, or other punctuation in the prefix).
  Mutual exclusion: bare `<name>` and series `<name>[]` cannot coexist.
  This guarantees `goto <name>` is unambiguous downstream.

Usage:
    docker exec -it 3d_nav_ros2 bash -lc "
      source /opt/ros/humble/setup.bash
      source /botbrain_ws/install/setup.bash
      python3 /g1_3d_nav_ros2/tools/gotop/capture_waypoints.py /tmp/waypoints.yaml
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

# Series detection: prefix is letters/underscores only (no digits or
# punctuation in prefix), suffix is one or more digits. Sorted by integer
# value of the suffix so office10 lands after office9.
SERIES_RE = re.compile(r"^([A-Za-z一-鿿][A-Za-z_一-鿿]*)(\d+)$")
SERIES_BRACKET_RE = re.compile(r"^([A-Za-z一-鿿][A-Za-z_一-鿿]*)\[\]$")


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
    """Returns (waypoints_dict, groups_dict). Preserves groups section
    untouched so that operators can hand-edit it in the yaml without
    fear of capture_waypoints clobbering their groups."""
    if not os.path.exists(path):
        return {}, {}
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        wps = data.get("waypoints", {}) or {}
        groups = data.get("groups", {}) or {}
        if isinstance(wps, list):
            print(f"  warning: {path} is in legacy list format; converting to dict.")
            wps = {wp.get("name", f"wp{i+1}"): {k: v for k, v in wp.items() if k != "name"}
                   for i, wp in enumerate(wps)}
        return wps, groups
    except Exception as e:
        print(f"  failed to load existing yaml: {e}")
        return {}, {}


def save_yaml(path, waypoints, groups):
    out = {"frame_id": SOURCE_FRAME, "waypoints": waypoints}
    if groups:
        out["groups"] = groups
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
    series = compute_series(waypoints)
    series_members = {label for entries in series.values() for _, label in entries}
    # Singles first (alphabetically), then folded series
    singles = sorted(label for label in waypoints if label not in series_members)
    for label in singles:
        print_pose_line(label, waypoints[label])
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
    print(f"  total: {len(waypoints)} ({len(singles)} singles, "
          f"{sum(len(v) for v in series.values())} in {len(series)} series)")


def cmd_del(waypoints, groups, label, path):
    # del office[]  -> delete all members of series
    bm = SERIES_BRACKET_RE.match(label)
    if bm:
        group = bm.group(1)
        series = compute_series(waypoints)
        if group not in series:
            print(f"  no such series: {group}[]")
            return
        members = [lab for _, lab in series[group]]
        for lab in members:
            del waypoints[lab]
            for g_name, g_list in list(groups.items()):
                if lab in g_list:
                    g_list.remove(lab)
        save_yaml(path, waypoints, groups)
        print(f"  deleted series {group}[] ({len(members)} pts: {', '.join(members)})")
        return
    if label not in waypoints:
        print(f"  no such label: {label!r}")
        return
    del waypoints[label]
    # Also remove from any group that referenced it
    for g_name, g_list in list(groups.items()):
        if label in g_list:
            g_list.remove(label)
    save_yaml(path, waypoints, groups)
    print(f"  deleted {label}")


def cmd_rename(waypoints, groups, old, new, path):
    if old not in waypoints:
        print(f"  no such label: {old!r}")
        return
    if not LABEL_RE.match(new):
        print(f"  invalid new label {new!r}: must match [A-Za-z][A-Za-z0-9_-]*")
        return
    if new in waypoints:
        print(f"  target label {new!r} already exists; del it first or pick another name")
        return
    waypoints[new] = waypoints.pop(old)
    # Update group references
    for g_name, g_list in groups.items():
        groups[g_name] = [new if x == old else x for x in g_list]
    save_yaml(path, waypoints, groups)
    print(f"  renamed {old} -> {new}")


def capture_one(buf, waypoints, label):
    """Sample current pose and stage it under `label` in `waypoints` (no
    save). Returns the captured pose dict, or None on failure / skip.
    Handles overwrite confirmation when the new pose is far from the old."""
    old = waypoints.get(label)
    pose = sample_pose(buf, SAMPLE_DURATION_S)
    if pose is None:
        print("  FAILED to read TF during sampling.")
        return None
    if old is not None:
        d = math.hypot(pose["x"] - old["x"], pose["y"] - old["y"])
        if d > OVERWRITE_PROMPT_DIST_M:
            ans = input(f"  {label!r} exists at ({old['x']:.2f},{old['y']:.2f}); "
                        f"new is ({pose['x']:.2f},{pose['y']:.2f}), {d:.2f}m away. "
                        f"Overwrite? (y/N): ").strip().lower()
            if ans not in ("y", "yes"):
                print("  skipped")
                return None
        else:
            print(f"  refreshing {label} ({d*100:.1f} cm shift)")
    waypoints[label] = pose
    return pose


def capture_series(buf, waypoints, groups, group, path):
    """Series capture loop. group is the bare prefix (e.g. 'office').
    Mutual exclusion: refuses if bare `group` already exists in waypoints.
    Picks up where the existing series left off (next free integer)."""
    if group in waypoints:
        print(f"  cannot start series {group}[]: bare label {group!r} already "
              f"exists. Run `del {group}` first to convert to series.")
        return
    series = compute_series(waypoints).get(group, [])
    next_idx = (max(idx for idx, _ in series) + 1) if series else 1
    if series:
        existing = ", ".join(lab for _, lab in series)
        print(f"  resuming series {group}[]: {len(series)} existing ({existing}); "
              f"new captures start at {group}{next_idx}.")
    else:
        print(f"  starting new series {group}[]; first capture will be {group}{next_idx}.")
    print(f"  type `q` to leave series mode and return to wp> (script keeps running).")
    while True:
        prompt = f"{group}[{next_idx}]> "
        try:
            line = input(prompt).strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if line in ("q", "quit"):
            return
        if line in ("list", "ls"):
            cmd_list(waypoints); continue
        if line == "":
            # Bare Enter: capture under the next index
            label = f"{group}{next_idx}"
        elif line.isdigit():
            # `5` → recapture/jump to office5 (allows redoing a specific one)
            label = f"{group}{int(line)}"
        else:
            print(f"  unknown command in series mode: {line!r}. "
                  f"Empty line = capture next ({group}{next_idx}); "
                  f"<n> = capture {group}<n>; q = leave series mode.")
            continue
        captured = capture_one(buf, waypoints, label)
        if captured is None:
            continue
        save_yaml(path, waypoints, groups)
        print(f"  captured ", end="")
        print_pose_line(label, captured)
        # Advance next_idx past whatever the user just picked, so empty-Enter
        # always lands on a fresh slot.
        next_idx = max(next_idx, parse_series_member(label)[1] + 1)


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
        print("     start. Run /g1_3d_nav_ros2/tools/launch.sh and wait for ALL 6 NODES RUNNING.")
        executor.shutdown(); rclpy.shutdown(); return 1
    print("OK")

    waypoints, groups = load_existing(args.output)
    if waypoints:
        print(f"\nLoaded {len(waypoints)} existing waypoints from {args.output}:")
        cmd_list(waypoints)
        if groups:
            print(f"  (groups defined: {', '.join(groups.keys())} — preserved on save)")
    else:
        print(f"\nStarting fresh; output -> {args.output}")

    print("\nCommands at prompt:")
    print("  <label>            capture current pose under that label")
    print("  <name>[]           enter series capture mode (office[] -> office1, office2, ...)")
    print("  list / ls          show all captured waypoints (series folded)")
    print("  del <label>        delete a waypoint")
    print("  del <name>[]       delete an entire series")
    print("  rename <old> <new> rename a waypoint (group refs updated too)")
    print("  q / quit           save and exit (Ctrl-D works too)\n")

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
            cmd_del(waypoints, groups, line[4:].strip(), args.output); continue
        if line.startswith("rename "):
            parts = line[7:].split()
            if len(parts) != 2:
                print("  usage: rename <old> <new>")
                continue
            cmd_rename(waypoints, groups, parts[0], parts[1], args.output); continue

        # Series capture: `<name>[]`
        bm = SERIES_BRACKET_RE.match(line)
        if bm:
            capture_series(buf, waypoints, groups, bm.group(1), args.output)
            continue

        label = line
        if not LABEL_RE.match(label):
            print(f"  invalid label {label!r}: must match [A-Za-z][A-Za-z0-9_-]*")
            continue

        # Single-point capture: refuse if a series with this label as prefix
        # already exists. Avoids ambiguous `goto office` downstream.
        if label not in waypoints:
            existing_series = compute_series(waypoints).get(label)
            if existing_series:
                members = ", ".join(lab for _, lab in existing_series)
                print(f"  cannot capture single {label!r}: series {label}[] already "
                      f"exists with {len(existing_series)} pts ({members}). Use "
                      f"{label}[] to extend, or `del {label}[]` to wipe the series first.")
                continue

        captured = capture_one(buf, waypoints, label)
        if captured is None:
            continue
        save_yaml(args.output, waypoints, groups)
        print(f"  captured ", end="")
        print_pose_line(label, captured)

    save_yaml(args.output, waypoints, groups)
    print(f"\nSaved {len(waypoints)} waypoints to {args.output}.")
    executor.shutdown()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
