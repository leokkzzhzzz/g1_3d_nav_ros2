#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interactive Marker End-Effector Goals — BotBrain adaptation.

Creates 6-DOF interactive markers in RViz for each hand and publishes
PoseStamped goals to the arm controller.

Changes from g1pilot:
  • All topics use BotBrain convention:  manipulation/hand_goal/{right,left}
  • Server namespace uses "manipulation_ee_goal_markers"
"""

import functools
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped, Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    Marker,
)
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)


class InteractiveMarkerEFF(Node):
    def __init__(self):
        super().__init__("multi_ee_goal_markers")

        # ----- parameters -----
        self.declare_parameter("fixed_frame", "pelvis")
        self.declare_parameter("spawn_rate_hz", 5.0)

        self.declare_parameter("right_tf_frame", "right_hand_point_contact")
        self.declare_parameter("right_topic", "manipulation/hand_goal/right")
        self.declare_parameter("right_scale", 0.05)

        self.declare_parameter("left_tf_frame", "left_hand_point_contact")
        self.declare_parameter("left_topic", "manipulation/hand_goal/left")
        self.declare_parameter("left_scale", 0.05)

        self.declare_parameter("publish_enabled_default", False)

        self.fixed_frame = self.get_parameter("fixed_frame").get_parameter_value().string_value
        self.spawn_dt = 1.0 / float(self.get_parameter("spawn_rate_hz").value)

        self.right_tf    = self.get_parameter("right_tf_frame").get_parameter_value().string_value
        self.right_topic = self.get_parameter("right_topic").get_parameter_value().string_value
        self.right_scale = float(self.get_parameter("right_scale").value)

        self.left_tf    = self.get_parameter("left_tf_frame").get_parameter_value().string_value
        self.left_topic = self.get_parameter("left_topic").get_parameter_value().string_value
        self.left_scale = float(self.get_parameter("left_scale").value)

        self.publish_default = bool(self.get_parameter("publish_enabled_default").value)

        # ----- interactive marker server -----
        self.server = InteractiveMarkerServer(self, "manipulation_ee_goal_markers")
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.qos = QoSProfile(depth=10)

        self.ee_publishers = {
            "right": self.create_publisher(PoseStamped, self.right_topic, self.qos),
            "left":  self.create_publisher(PoseStamped, self.left_topic, self.qos),
        }

        self.publish_enabled = {"right": self.publish_default, "left": self.publish_default}
        self.current_pose    = {"right": None, "left": None}

        self.menu_handlers   = {"right": self._build_menu_handler(), "left": self._build_menu_handler()}
        self.menu_entry_ids  = {"right": {}, "left": {}}
        self.marker_spawned  = {"right": False, "left": False}

        self.timer = self.create_timer(self.spawn_dt, self._try_spawn_missing)

    # ---------------------------------------------------------------
    # Menu
    # ---------------------------------------------------------------
    def _build_menu_handler(self) -> MenuHandler:
        mh = MenuHandler()
        toggle_id = mh.insert("Enable publishing", callback=self._menu_cb)
        reset_id  = mh.insert("Reset to TF pose",  callback=self._menu_cb)
        mh.setCheckState(toggle_id, MenuHandler.CHECKED if self.publish_default else MenuHandler.UNCHECKED)
        mh._toggle_id = toggle_id  # noqa: access private field for convenience
        mh._reset_id  = reset_id
        return mh

    # ---------------------------------------------------------------
    # Spawning
    # ---------------------------------------------------------------
    def _try_spawn_missing(self):
        if not self.marker_spawned["right"]:
            self._try_spawn_one("right", self.right_tf, self.right_scale)
        if not self.marker_spawned["left"]:
            self._try_spawn_one("left", self.left_tf, self.left_scale)

    def _try_spawn_one(self, side, tf_frame, scale):
        try:
            trans = self.tf_buffer.lookup_transform(self.fixed_frame, tf_frame, rclpy.time.Time())
            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation
            self.current_pose[side] = pose
            self._spawn_marker(side, scale, pose)
            self.marker_spawned[side] = True
            self.get_logger().info(f"Spawned {side} hand marker from TF '{tf_frame}' in '{self.fixed_frame}'")
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass  # silently retry on next timer tick

    def _spawn_marker(self, side, cube_size, pose: Pose):
        name = f"{side}_hand_goal"

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.fixed_frame
        int_marker.name = name
        int_marker.description = f"{side.title()} Hand Goal"
        int_marker.scale = 0.2
        int_marker.pose = pose

        box = Marker()
        box.type = Marker.CUBE
        box.scale.x = cube_size
        box.scale.y = cube_size
        box.scale.z = cube_size

        if self.publish_enabled[side]:
            if side == "right":
                box.color.r, box.color.g, box.color.b, box.color.a = 0.2, 0.8, 0.2, 1.0
            else:
                box.color.r, box.color.g, box.color.b, box.color.a = 0.2, 0.2, 0.8, 1.0
        else:
            box.color.r, box.color.g, box.color.b, box.color.a = 0.5, 0.5, 0.5, 1.0

        always = InteractiveMarkerControl()
        always.always_visible = True
        always.markers.append(box)
        int_marker.controls.append(always)

        menu_ctrl = InteractiveMarkerControl()
        menu_ctrl.interaction_mode = InteractiveMarkerControl.MENU
        menu_ctrl.name = "menu"
        int_marker.controls.append(menu_ctrl)

        self._add_6dof_controls(int_marker)

        self.server.insert(int_marker)
        cb = functools.partial(self._feedback_cb, ee_name=side)
        self.server.setCallback(int_marker.name, cb)

        mh = self.menu_handlers[side]
        mh.apply(self.server, int_marker.name)
        self.menu_entry_ids[side] = {"toggle": mh._toggle_id, "reset": mh._reset_id}
        self.server.applyChanges()

    @staticmethod
    def _add_6dof_controls(int_marker: InteractiveMarker):
        axes = {
            "x": (1.0, 0.0, 0.0, 1.0),
            "y": (0.0, 1.0, 0.0, 1.0),
            "z": (0.0, 0.0, 1.0, 1.0),
        }
        for axis, (ox, oy, oz, ow) in axes.items():
            for mode in (InteractiveMarkerControl.ROTATE_AXIS, InteractiveMarkerControl.MOVE_AXIS):
                c = InteractiveMarkerControl()
                c.orientation.x = float(ox)
                c.orientation.y = float(oy)
                c.orientation.z = float(oz)
                c.orientation.w = float(ow)
                c.name = f"{'ROT' if mode == InteractiveMarkerControl.ROTATE_AXIS else 'MOV'}_{axis}"
                c.interaction_mode = mode
                int_marker.controls.append(c)

    # ---------------------------------------------------------------
    # Feedback
    # ---------------------------------------------------------------
    def _feedback_cb(self, feedback, ee_name: str):
        self.current_pose[ee_name] = feedback.pose
        if not self.publish_enabled.get(ee_name, True):
            return
        pub = self.ee_publishers.get(ee_name)
        if pub is None:
            return

        ps = PoseStamped()
        ps.header = feedback.header
        ps.header.frame_id = self.fixed_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = feedback.pose
        pub.publish(ps)

    def _menu_cb(self, feedback):
        marker_name = feedback.marker_name or ""
        if marker_name.startswith("right_"):
            side = "right"
        elif marker_name.startswith("left_"):
            side = "left"
        else:
            return

        entry_id = feedback.menu_entry_id
        ids = self.menu_entry_ids[side]
        mh  = self.menu_handlers[side]

        if entry_id == ids["toggle"]:
            new_state = not self.publish_enabled[side]
            self.publish_enabled[side] = new_state
            mh.setCheckState(ids["toggle"], MenuHandler.CHECKED if new_state else MenuHandler.UNCHECKED)
            mh.reApply(self.server)
            self._recolor_marker(side)
            self.server.applyChanges()
            self.get_logger().info(f"[{side}] Publishing {'ENABLED' if new_state else 'DISABLED'}")

        elif entry_id == ids["reset"]:
            tf_frame = self.right_tf if side == "right" else self.left_tf
            self._reset_marker_to_tf(side, tf_frame)
            self.server.applyChanges()
            self.get_logger().info(f"[{side}] Reset to TF '{tf_frame}'")

    def _recolor_marker(self, side: str):
        pose = self.current_pose.get(side)
        if pose is None:
            return
        name = f"{side}_hand_goal"
        try:
            self.server.erase(name)
        except Exception:
            pass
        cube_size = self.right_scale if side == "right" else self.left_scale
        self._spawn_marker(side, cube_size, pose)

    def _reset_marker_to_tf(self, side: str, tf_frame: str):
        try:
            trans = self.tf_buffer.lookup_transform(self.fixed_frame, tf_frame, rclpy.time.Time())
            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation
            self.current_pose[side] = pose
            self.server.setPose(f"{side}_hand_goal", pose)
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warning(f"Cannot reset {side}: TF {self.fixed_frame}->{tf_frame} not available yet.")


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveMarkerEFF()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
