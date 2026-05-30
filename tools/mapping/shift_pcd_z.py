#!/usr/bin/env python3
"""Shift the z-coordinate of every point in a binary PCD by a constant.

Why this exists: D-012 (see docs/DECISIONS.md) reframes the localization PCD
so its floor sits at z=0. Mapping pipelines (HongTu, FAST-LIO) write PCDs whose
z=0 is at the LiDAR's startup height, not the floor. Before such a PCD can
serve as a localization map under D-012, every point's z must be offset by
the URDF `base_footprint -> mid360_link` height (1.247 m on G1 with the
standard MID360 mount — DOMAIN.md "ground offset constant").

Preserves the PCD header bit-for-bit and all non-z fields (intensity, normals,
curvature). Writes atomically via a sibling tempfile + rename.

Usage:
    python3 shift_pcd_z.py <pcd_path> [shift_z]

Defaults to shift_z=1.247 (G1 ground offset constant). Pass a different value
for non-G1 robots or modified mounts.
"""
import os
import sys

import numpy as np


def shift_pcd_z(pcd_path: str, shift_z: float) -> None:
    with open(pcd_path, "rb") as f:
        raw = f.read()

    header_end = raw.index(b"\nDATA ")
    data_line_end = raw.index(b"\n", header_end + 1)
    header_bytes = raw[: data_line_end + 1]
    body_bytes = raw[data_line_end + 1 :]

    header_text = header_bytes.decode()

    def get_line(prefix: str):
        for line in header_text.splitlines():
            if line.startswith(prefix):
                return line.split()[1:]
        raise KeyError(prefix)

    fields = get_line("FIELDS")
    sizes = [int(x) for x in get_line("SIZE")]
    types = get_line("TYPE")
    counts = [int(x) for x in get_line("COUNT")]
    points = int(get_line("POINTS")[0])
    fmt = get_line("DATA")[0]

    if fmt != "binary":
        raise ValueError(f"only binary PCD supported, got {fmt}")

    np_t = {
        ("F", 4): "f4",
        ("F", 8): "f8",
        ("U", 1): "u1",
        ("U", 2): "u2",
        ("U", 4): "u4",
        ("I", 2): "i2",
        ("I", 4): "i4",
    }
    descr = []
    for fld, t, s, c in zip(fields, types, sizes, counts):
        descr.append((fld, np_t[(t, s)], (c,) if c > 1 else ()))
    dtype = np.dtype(descr)
    expected = points * dtype.itemsize
    if len(body_bytes) != expected:
        raise ValueError(f"body size {len(body_bytes)} != expected {expected}")

    arr = np.frombuffer(body_bytes, dtype=dtype).copy()
    z_before = arr["z"].astype(np.float64)
    arr["z"] = (z_before + shift_z).astype(arr["z"].dtype)
    z_after = arr["z"].astype(np.float64)

    print(f"=== {pcd_path} ===")
    print(f"  fields={fields}, points={points:,}")
    print(
        f"  z 1pct  {np.percentile(z_before, 1):+.4f} -> "
        f"{np.percentile(z_after, 1):+.4f}"
    )
    print(
        f"  z 5pct  {np.percentile(z_before, 5):+.4f} -> "
        f"{np.percentile(z_after, 5):+.4f}"
    )
    print(
        f"  z med   {np.percentile(z_before, 50):+.4f} -> "
        f"{np.percentile(z_after, 50):+.4f}"
    )

    new_body = arr.tobytes()
    if len(new_body) != expected:
        raise RuntimeError("post-shift body length mismatch")
    tmp = pcd_path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(header_bytes)
        f.write(new_body)
    os.replace(tmp, pcd_path)
    print(f"  wrote {os.path.getsize(pcd_path):,} bytes")


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__, file=sys.stderr)
        return 2
    pcd_path = sys.argv[1]
    shift_z = float(sys.argv[2]) if len(sys.argv) >= 3 else 1.247
    if not os.path.exists(pcd_path):
        print(f"missing: {pcd_path}", file=sys.stderr)
        return 1
    shift_pcd_z(pcd_path, shift_z)
    return 0


if __name__ == "__main__":
    sys.exit(main())
