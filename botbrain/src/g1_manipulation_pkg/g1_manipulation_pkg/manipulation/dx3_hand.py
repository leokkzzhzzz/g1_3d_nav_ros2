#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DX3 Dexterous-hand controller — BotBrain adaptation.

Changes from g1pilot original:
  • Removed ``astroviz_interfaces`` dependency — gripper motor states are
    published as ``sensor_msgs/JointState`` (standard, zero external deps).
  • All topic names use BotBrain convention:
    ``manipulation/dx3/hand_action/{right,left}``   (subscribe, String)
    ``manipulation/dx3/{left,right}/motor_state``    (publish, JointState)
  • Reads ``robot_config.yaml`` for network interface.
"""

import os
import yaml

import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from unitree_sdk2py.core.channel import (
    ChannelPublisher,
    ChannelSubscriber,
    ChannelFactoryInitialize,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_

# Gripper preset values (from g1pilot)
CLOSE_RIGHT_VALUES = [-0.10, 0.63, -1.74, 1.06, 0.95, 0.91, 1.22]
CLOSE_LEFT_VALUES  = [0.04, -0.04, 1.51, -1.10, -1.47, -1.13, -1.23]
OPEN_VALUES        = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class DX3Controller(Node):
    def __init__(self):
        super().__init__("dx3_hand_controller")

        # ----- parameters -----
        self.declare_parameter("interface", self._read_interface())
        self.declare_parameter("arm_controlled", "both")

        interface      = self.get_parameter("interface").get_parameter_value().string_value
        arm_controlled = self.get_parameter("arm_controlled").get_parameter_value().string_value

        # ----- ROS publishers (BotBrain namespace, standard JointState) -----
        self.left_gripper_state_publisher = self.create_publisher(
            JointState, "manipulation/dx3/left/motor_state", QoSProfile(depth=10)
        )
        self.right_gripper_state_publisher = self.create_publisher(
            JointState, "manipulation/dx3/right/motor_state", QoSProfile(depth=10)
        )

        self.right_action = None
        self.left_action  = None
        self.right_target = OPEN_VALUES
        self.left_target  = OPEN_VALUES
        self.total_motors = 7
        self.send_commands = True

        # ----- DDS -----
        ChannelFactoryInitialize(0, interface)

        if arm_controlled in ["right", "both"]:
            self.right_pub = ChannelPublisher("rt/dex3/right/cmd", HandCmd_)
            self.right_pub.Init()
            self.right_sub = ChannelSubscriber("rt/dex3/right/state", HandState_)
            self.right_sub.Init(self.right_callback)
            self.create_subscription(
                String, "manipulation/dx3/hand_action/right", self.right_action_callback, 10
            )

        if arm_controlled in ["left", "both"]:
            self.left_pub = ChannelPublisher("rt/dex3/left/cmd", HandCmd_)
            self.left_pub.Init()
            self.left_sub = ChannelSubscriber("rt/dex3/left/state", HandState_)
            self.left_sub.Init(self.left_callback)
            self.create_subscription(
                String, "manipulation/dx3/hand_action/left", self.left_action_callback, 10
            )

        self.create_timer(0.05, self.publish_commands)
        self.get_logger().info(
            f"DX3Controller started — interface={interface}, arms={arm_controlled}"
        )

    # ---------------------------------------------------------------
    # Config helper
    # ---------------------------------------------------------------
    @staticmethod
    def _read_interface() -> str:
        cfg_path = "/botbrain_ws/robot_config.yaml"
        if not os.path.isfile(cfg_path):
            return "eth0"
        try:
            with open(cfg_path, "r") as f:
                cfg = yaml.safe_load(f).get("robot_configuration", {})
            return cfg.get("network_interface", "eth0")
        except Exception:
            return "eth0"

    # ---------------------------------------------------------------
    # Action callbacks
    # ---------------------------------------------------------------
    def right_action_callback(self, msg: String):
        if msg.data not in ("open", "close"):
            return
        if msg.data != self.right_action:
            self.right_action = msg.data
            self.right_target = CLOSE_RIGHT_VALUES if msg.data == "close" else OPEN_VALUES
            self.get_logger().info(f"Right hand → {msg.data}")

    def left_action_callback(self, msg: String):
        if msg.data not in ("open", "close"):
            return
        if msg.data != self.left_action:
            self.left_action = msg.data
            self.left_target = CLOSE_LEFT_VALUES if msg.data == "close" else OPEN_VALUES
            self.get_logger().info(f"Left hand → {msg.data}")

    # ---------------------------------------------------------------
    # DDS state callbacks → publish as standard JointState
    # ---------------------------------------------------------------
    def _publish_gripper_state(self, dds_msg: HandState_, side: str):
        """Convert HandState_ to sensor_msgs/JointState and publish."""
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [f"{side}_motor_{i}" for i in range(len(dds_msg.motor_state))]
        js.position = [float(dds_msg.motor_state[i].q) for i in range(len(dds_msg.motor_state))]
        js.velocity = [float(dds_msg.motor_state[i].dq) for i in range(len(dds_msg.motor_state))]
        js.effort   = []  # not available from HandState_
        pub = self.left_gripper_state_publisher if side == "left" else self.right_gripper_state_publisher
        pub.publish(js)

    def left_callback(self, msg: HandState_):
        self._publish_gripper_state(msg, "left")

    def right_callback(self, msg: HandState_):
        self._publish_gripper_state(msg, "right")

    # ---------------------------------------------------------------
    # Command publishing (20 Hz)
    # ---------------------------------------------------------------
    def create_cmd(self, values):
        cmd = unitree_hg_msg_dds__HandCmd_()
        for i in range(self.total_motors):
            cmd.motor_cmd[i].mode = 0
            cmd.motor_cmd[i].q    = values[i]
            cmd.motor_cmd[i].dq   = 0.0
            cmd.motor_cmd[i].tau  = 0.0
            cmd.motor_cmd[i].kp   = 1.5
            cmd.motor_cmd[i].kd   = 0.2
        return cmd

    def publish_commands(self):
        if not self.send_commands:
            return
        if hasattr(self, "right_pub") and self.right_action is not None:
            self.right_pub.Write(self.create_cmd(self.right_target))
        if hasattr(self, "left_pub") and self.left_action is not None:
            self.left_pub.Write(self.create_cmd(self.left_target))


def main(args=None):
    # ── 防止 DDS domain 冲突 (monkey-patch) ──
    # 见 arm_controller.py main() 中的详细注释。
    # 用空壳 Domain 替换 cyclonedds.domain.Domain，
    # 使 ChannelFactoryInitialize 不真正调 dds_create_domain。
    # domain 0 统一由 rmw_cyclonedds_cpp 通过 CYCLONEDDS_URI 创建。
    # monkey-patch：见 arm_controller.py 中的详细注释。
    # 必须 patch channel.py 模块本地的 Domain 绑定。
    import unitree_sdk2py.core.channel as _ch_mod

    class _NoOpDomain:
        def __init__(self, *a, **kw): pass
        def close(self): pass
        def __del__(self): pass

    _ch_mod.Domain = _NoOpDomain

    # ChannelFactoryInitialize 已在 DX3Controller.__init__ 中
    # （super().__init__ 之后）调用，此时 rmw 已建好 domain 0。

    rclpy.init(args=args)
    node = DX3Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
