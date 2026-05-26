#!/usr/bin/env python3
"""ROS2 port of deepglint's ground_cloud_accumulator (ROS1) — produces a
2D OccupancyGrid by classifying body-frame point cloud points into ground
(FREE) and obstacle (OCCUPIED) by their z, transforming to map frame, and
accumulating into a fixed-resolution grid that grows on demand.

Subscribes:  /cloud_registered_body_1  (sensor_msgs/PointCloud2, body frame)
Publishes:   /accumulated_grid         (nav_msgs/OccupancyGrid, map frame)

The companion to fast_lio mapping. fast_lio dumps the 3D PCD via the
/map_save service; map_saver_cli on the OccupancyGrid topic dumps the 2D
PGM + yaml. Together they form the ROS2-native equivalent of the ROS1
fast_lio + ground_cloud_accumulator mapping pipeline.

Usage (inside 3d_nav_ros2 container, requires launch.sh up so /tf and
/cloud_registered_body_1 flow):
    python3 /tmp/grid_accumulator.py \\
        --resolution 0.05 --ground-z 0.15 --obstacle-z 0.25 \\
        --rate 2.0

To save the produced grid to disk (separate window, after enough mapping
data has been accumulated):
    ros2 run nav2_map_server map_saver_cli \\
        -t /accumulated_grid -f /tmp/accumulated_grid

Defaults match the ROS1 ground_cloud_accumulator parameters used in the
2026-05-21 mapping run (the one that produced /root/maps/accumulated_grid.*).
"""
import argparse
import math
import os
import sys
import time
from threading import Lock, Thread

# RMW must match the running stack (rmw_zenoh_cpp). See capture_waypoints.py
# for the rationale. setdefault leaves room for a shell-level override.
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
os.environ.setdefault(
    "ZENOH_CONFIG_OVERRIDE",
    'mode="client";connect/endpoints=["tcp/127.0.0.1:7448"]',
)

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


FREE = 0
OCCUPIED = 100
UNKNOWN = -1


class GridAccumulator(Node):
    def __init__(self, args):
        super().__init__("grid_accumulator")
        self.res = args.resolution
        self.ground_z = args.ground_z
        self.obstacle_z = args.obstacle_z
        self.rate = args.rate
        self.body_frame = args.body_frame
        self.map_frame = args.map_frame
        self.cloud_topic = args.cloud_topic
        self.grid_topic = args.grid_topic

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        self.grid = None             # int8 numpy array, shape (h, w)
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.lock = Lock()

        self.frames = 0
        self.ground_pts = 0
        self.obs_pts = 0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            PointCloud2, self.cloud_topic, self.cloud_cb, sensor_qos)
        self.pub = self.create_publisher(OccupancyGrid, self.grid_topic, 1)
        self.create_timer(1.0 / self.rate, self.publish_grid)
        self.create_timer(5.0, self.log_stats)

        self.get_logger().info(
            f"grid_accumulator: res={self.res}m, "
            f"ground_z<{self.ground_z}, obstacle_z>{self.obstacle_z}, "
            f"sub={self.cloud_topic}, pub={self.grid_topic}")

    def log_stats(self):
        if self.grid is None:
            self.get_logger().info(
                f"frames={self.frames} (no grid yet; waiting for first cloud + TF)")
        else:
            self.get_logger().info(
                f"frames={self.frames} ground={self.ground_pts} obs={self.obs_pts} "
                f"grid={self.grid.shape[1]}x{self.grid.shape[0]} "
                f"origin=({self.origin_x:.2f},{self.origin_y:.2f})")

    def cloud_cb(self, msg: PointCloud2):
        try:
            tf = self.tf_buf.lookup_transform(
                self.map_frame, msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return  # TF not yet available
        pts = np.array(list(pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float64)
        if pts.size == 0:
            return
        # Reshape — sensor_msgs_py returns structured array in some versions
        if pts.dtype.names is not None:
            pts = np.column_stack([pts["x"], pts["y"], pts["z"]])
        else:
            pts = pts.reshape(-1, 3)

        # Classify by z in body frame BEFORE transform.
        z = pts[:, 2]
        ground_mask = z < self.ground_z
        obs_mask = z > self.obstacle_z

        # Transform body → map: rotate + translate
        q = tf.transform.rotation
        t = tf.transform.translation
        # Rotation matrix from quaternion
        xx, yy, zz, ww = q.x, q.y, q.z, q.w
        rot = np.array([
            [1 - 2 * (yy * yy + zz * zz), 2 * (xx * yy - zz * ww), 2 * (xx * zz + yy * ww)],
            [2 * (xx * yy + zz * ww), 1 - 2 * (xx * xx + zz * zz), 2 * (yy * zz - xx * ww)],
            [2 * (xx * zz - yy * ww), 2 * (yy * zz + xx * ww), 1 - 2 * (xx * xx + yy * yy)],
        ])
        pts_map = pts @ rot.T + np.array([t.x, t.y, t.z])

        with self.lock:
            self._ingest(pts_map[:, 0], pts_map[:, 1], ground_mask, obs_mask)
            self.frames += 1
            self.ground_pts += int(ground_mask.sum())
            self.obs_pts += int(obs_mask.sum())

    def _ingest(self, xs, ys, ground_mask, obs_mask):
        # Determine bbox of new points
        x_min, x_max = float(xs.min()), float(xs.max())
        y_min, y_max = float(ys.min()), float(ys.max())

        if self.grid is None:
            # Initialise grid centred near these points with small margin
            margin = 5.0
            self.origin_x = x_min - margin
            self.origin_y = y_min - margin
            w = int(math.ceil((x_max - x_min + 2 * margin) / self.res))
            h = int(math.ceil((y_max - y_min + 2 * margin) / self.res))
            self.grid = np.full((h, w), UNKNOWN, dtype=np.int8)
        else:
            # Grow grid if needed (auto_resize)
            self._grow_to_include(x_min, y_min, x_max, y_max, margin=5.0)

        # Convert to indices
        ix = ((xs - self.origin_x) / self.res).astype(np.int32)
        iy = ((ys - self.origin_y) / self.res).astype(np.int32)
        h, w = self.grid.shape
        valid = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        ix, iy = ix[valid], iy[valid]
        ground_mask = ground_mask[valid]
        obs_mask = obs_mask[valid]

        # FREE first (so OCCUPIED can win on overlapping cells in same frame)
        free_idx = (iy[ground_mask], ix[ground_mask])
        # Only mark as FREE cells that are still UNKNOWN — don't undo OCCUPIED
        free_undecided = self.grid[free_idx] == UNKNOWN
        self.grid[free_idx[0][free_undecided], free_idx[1][free_undecided]] = FREE
        # OCCUPIED wins everywhere
        self.grid[iy[obs_mask], ix[obs_mask]] = OCCUPIED

    def _grow_to_include(self, x_min, y_min, x_max, y_max, margin):
        h, w = self.grid.shape
        cur_x_max = self.origin_x + w * self.res
        cur_y_max = self.origin_y + h * self.res
        need_left = max(0.0, self.origin_x - (x_min - margin))
        need_right = max(0.0, (x_max + margin) - cur_x_max)
        need_bottom = max(0.0, self.origin_y - (y_min - margin))
        need_top = max(0.0, (y_max + margin) - cur_y_max)
        if need_left == need_right == need_bottom == need_top == 0:
            return
        pad_l = int(math.ceil(need_left / self.res))
        pad_r = int(math.ceil(need_right / self.res))
        pad_b = int(math.ceil(need_bottom / self.res))
        pad_t = int(math.ceil(need_top / self.res))
        new_w = w + pad_l + pad_r
        new_h = h + pad_b + pad_t
        new_grid = np.full((new_h, new_w), UNKNOWN, dtype=np.int8)
        new_grid[pad_b:pad_b + h, pad_l:pad_l + w] = self.grid
        self.grid = new_grid
        self.origin_x -= pad_l * self.res
        self.origin_y -= pad_b * self.res

    def publish_grid(self):
        with self.lock:
            if self.grid is None:
                return
            h, w = self.grid.shape
            data = self.grid.flatten().tolist()
            origin_x, origin_y = self.origin_x, self.origin_y
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.res
        msg.info.width = w
        msg.info.height = h
        pose = Pose()
        pose.position.x = origin_x
        pose.position.y = origin_y
        pose.orientation.w = 1.0
        msg.info.origin = pose
        msg.data = data
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--ground-z", type=float, default=0.15)
    ap.add_argument("--obstacle-z", type=float, default=0.25)
    ap.add_argument("--rate", type=float, default=2.0)
    ap.add_argument("--body-frame", default="body")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--cloud-topic", default="/cloud_registered_body_1")
    ap.add_argument("--grid-topic", default="/accumulated_grid")
    args = ap.parse_args()

    rclpy.init()
    node = GridAccumulator(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f"shutting down. final stats: frames={node.frames} "
            f"ground={node.ground_pts} obs={node.obs_pts}")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
