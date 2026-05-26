#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
键盘增量控制手臂位置 — 无需 GUI，SSH 终端即可使用。

用法:
  ros2 run g1_manipulation_pkg arm_teleop_keyboard

按键:
  w/s  — 前/后 (x)
  a/d  — 左/右 (y)
  q/e  — 上/下 (z)
  1/2  — 切换 左臂/右臂
  h    — 归位 (home)
  o    — 启用手臂
  p    — 禁用手臂
  +/-  — 调节步长
  Ctrl+C — 退出
"""

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

HELP_TEXT = """
╔══════════════════════════════════════════╗
║      手臂键盘遥控 (Arm Teleop)          ║
╠══════════════════════════════════════════╣
║  w/s : 前进/后退  (x)                   ║
║  a/d : 向左/向右  (y)                   ║
║  q/e : 上升/下降  (z)                   ║
║  u/j : 偏航 yaw+/-                      ║
║  i/k : 俯仰 pitch+/-                   ║
║  o/l : 滚转 roll+/-  (未启用)           ║
║  1   : 切换到左臂                        ║
║  2   : 切换到右臂                        ║
║  h   : 归位 (Home)                      ║
║  [   : 启用手臂 (Enable)               ║
║  ]   : 禁用手臂 (Disable)              ║
║  +/= : 增大步长                          ║
║  -   : 减小步长                          ║
║  Ctrl+C : 退出                           ║
╚══════════════════════════════════════════╝
"""

# Default safe starting positions (pelvis frame)
DEFAULT_POS = {
    "right": {"x": 0.30, "y": -0.20, "z": 0.10},
    "left":  {"x": 0.30, "y":  0.20, "z": 0.10},
}


def get_key(settings):
    """Read a single keypress (non-blocking style)."""
    tty.setraw(sys.stdin.fileno())
    try:
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class ArmTeleopKeyboard(Node):
    def __init__(self):
        super().__init__("arm_teleop_keyboard")

        self.right_pub = self.create_publisher(
            PoseStamped, "manipulation/hand_goal/right", 10
        )
        self.left_pub = self.create_publisher(
            PoseStamped, "manipulation/hand_goal/left", 10
        )
        self.enable_pub = self.create_publisher(
            Bool, "manipulation/enabled", 10
        )
        self.home_pub = self.create_publisher(
            Bool, "manipulation/home", 10
        )

        # State
        self.active_arm = "right"  # "right" or "left"
        self.step = 0.02  # metres per keypress
        self.pos = {
            "right": dict(DEFAULT_POS["right"]),
            "left":  dict(DEFAULT_POS["left"]),
        }
        # Orientation as quaternion (w, x, y, z) — start with identity
        self.ori = {
            "right": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
            "left":  {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
        }

    def publish_goal(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "pelvis"
        p = self.pos[self.active_arm]
        o = self.ori[self.active_arm]
        msg.pose.position.x = p["x"]
        msg.pose.position.y = p["y"]
        msg.pose.position.z = p["z"]
        msg.pose.orientation.w = o["w"]
        msg.pose.orientation.x = o["x"]
        msg.pose.orientation.y = o["y"]
        msg.pose.orientation.z = o["z"]

        if self.active_arm == "right":
            self.right_pub.publish(msg)
        else:
            self.left_pub.publish(msg)

    def print_status(self):
        p = self.pos[self.active_arm]
        arm_str = "右臂 (Right)" if self.active_arm == "right" else "左臂 (Left)"
        print(
            f"\r  [{arm_str}]  "
            f"x={p['x']:+.3f}  y={p['y']:+.3f}  z={p['z']:+.3f}  "
            f"step={self.step:.3f}m    ",
            end="",
            flush=True,
        )

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        print(HELP_TEXT)
        print("  当前活跃: 右臂 (Right)")
        self.print_status()

        try:
            while True:
                key = get_key(settings)
                p = self.pos[self.active_arm]
                changed = False

                if key == "w":
                    p["x"] += self.step; changed = True
                elif key == "s":
                    p["x"] -= self.step; changed = True
                elif key == "a":
                    p["y"] += self.step; changed = True
                elif key == "d":
                    p["y"] -= self.step; changed = True
                elif key == "q":
                    p["z"] += self.step; changed = True
                elif key == "e":
                    p["z"] -= self.step; changed = True
                elif key == "1":
                    self.active_arm = "left"
                    print(f"\n  切换到: 左臂 (Left)")
                elif key == "2":
                    self.active_arm = "right"
                    print(f"\n  切换到: 右臂 (Right)")
                elif key == "h":
                    msg = Bool(); msg.data = True
                    self.home_pub.publish(msg)
                    print("\n  >>> 归位指令已发送 (Home)")
                elif key == "[":
                    msg = Bool(); msg.data = True
                    self.enable_pub.publish(msg)
                    print("\n  >>> 手臂已启用 (Enabled)")
                elif key == "]":
                    msg = Bool(); msg.data = False
                    self.enable_pub.publish(msg)
                    print("\n  >>> 手臂已禁用 (Disabled)")
                elif key in ("+", "="):
                    self.step = min(0.10, self.step + 0.005)
                elif key == "-":
                    self.step = max(0.005, self.step - 0.005)
                elif key == "\x03":  # Ctrl+C
                    break

                if changed:
                    self.publish_goal()

                self.print_status()

        except Exception as e:
            print(f"\nError: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            print("\n  退出手臂遥控。")


def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleopKeyboard()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
