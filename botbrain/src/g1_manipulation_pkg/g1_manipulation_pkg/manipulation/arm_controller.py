#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArmController — ROS 2 node for G1 real-time arm control in BotBrain.

Migrated from g1pilot/manipulation/arm_controller.py with the following
BotBrain-specific adaptations:

  • All topics use BotBrain namespace convention:  /{robot_name}/manipulation/...
  • **Mode B — Grasp-Stop-Nav**: when arms are enabled, the node publishes
    zero-velocity on a high-priority twist_mux input to stop the base, ensuring
    safe arm operations.  Navigation goals remain intact; they resume once
    arm control is disabled.
  • The node reads *robot_config.yaml* for robot_name/network_interface to be
    consistent with the rest of BotBrain.
  • Does NOT write ``cmd_vel_out`` — only uses a dedicated ``manipulation_vel``
    input that twist_mux prioritises above navigation but below emergency stop.
"""

import os
import time
import threading
import math
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Point, Twist
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, ColorRGBA
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

import pinocchio as pin
from pinocchio import SE3

from g1_manipulation_pkg.utils.joints_names import (
    JOINT_NAMES_ROS,
    JOINT_LIMITS_RAD,
    RIGHT_JOINT_INDICES_LIST,
    LEFT_JOINT_INDICES_LIST,
)
from g1_manipulation_pkg.utils.ik_solver import G1IKSolver
from g1_manipulation_pkg.utils.common import (
    MotorState,
    G1_29_JointArmIndex,
    G1_29_JointWristIndex,
    G1_29_JointWeakIndex,
    G1_29_JointIndex,
    DataBuffer,
)

from unitree_sdk2py.core.channel import (
    ChannelPublisher,
    ChannelSubscriber,
    ChannelFactoryInitialize,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

# ---------------------------------------------------------------------------
# Workspace bounding boxes (pelvis frame) — identical to g1pilot
# ---------------------------------------------------------------------------
WORKSPACE = {
    "frame": "pelvis",
    "left_arm": {
        "left_bottom_front":  [0.33, 0.24, 0.02],
        "right_bottom_front": [0.33, 0.07, 0.02],
        "left_bottom_back":   [0.16, 0.24, 0.02],
        "right_bottom_back":  [0.16, 0.07, 0.02],
        "right_top_back":     [0.07, 0.20, 0.20],
        "left_top_back":      [0.07, 0.47, 0.20],
        "right_top_front":    [0.45, 0.11, 0.20],
        "left_top_front":     [0.41, 0.30, 0.20],
    },
    "right_arm": {
        "left_bottom_front":  [0.33, -0.24, 0.02],
        "right_bottom_front": [0.33, -0.07, 0.02],
        "left_bottom_back":   [0.16, -0.24, 0.02],
        "right_bottom_back":  [0.16, -0.07, 0.02],
        "right_top_back":     [0.07, -0.20, 0.20],
        "left_top_back":      [0.07, -0.47, 0.20],
        "right_top_front":    [0.45, -0.11, 0.20],
        "left_top_front":     [0.41, -0.30, 0.20],
    },
}


def _mat_to_quat_wxyz(R: np.ndarray):
    q = pin.Quaternion(R)
    return np.array([q.w, q.x, q.y, q.z])


def _quat_wxyz_to_matrix(qwxyz):
    w, x, y, z = qwxyz
    return pin.Quaternion(w, x, y, z).matrix()


# ===================================================================
class ArmController(Node):
    """BotBrain G1 arm controller with Mode-B (grasp→stop base)."""

    # ---------------------------------------------------------------
    # construction
    # ---------------------------------------------------------------
    def __init__(self):
        super().__init__("arm_controller")
        self.get_logger().info("=== BotBrain Arm Controller starting ===")

        # ----- read BotBrain robot_config.yaml for robot_name & interface -----
        self._read_botbrain_config()

        # ----- ROS parameters (same semantics as g1pilot) -----
        self.declare_parameter("use_robot", True)
        self.declare_parameter("interface", self._network_interface)
        self.declare_parameter("arm_velocity_limit", 5.0)
        self.declare_parameter("rate_hz", 250.0)
        self.declare_parameter("ik_world_frame", "pelvis")
        self.declare_parameter("ik_alpha", 0.2)
        self.declare_parameter("ik_goal_filter_alpha", 0.25)
        self.declare_parameter("ik_orientation_mode", "full")
        self.declare_parameter("ik_max_ori_step_rad", 0.35)
        self.declare_parameter("ee_auto_calibrate", True)
        self.declare_parameter("ee_offset_right_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("ee_offset_right_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("ee_offset_left_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("ee_offset_left_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("auto_reissue_goals", True)
        self.declare_parameter("goal_pos_tol", 0.01)
        self.declare_parameter("goal_ori_tol_deg", 3.0)

        # BotBrain Mode-B: zero-vel topic name for twist_mux
        self.declare_parameter("stop_nav_topic", "manipulation_vel")

        # 禁用手臂时是否交还上身控制权 (mode_pr=0)
        # true  = 交还，运控接管手臂回到默认站姿（默认）
        # false = 不交还，手臂保持当前位置（仍由 arm_sdk 维持力矩）
        self.declare_parameter("release_on_disable", True)

        self.use_robot    = bool(self.get_parameter("use_robot").value)
        self.interface    = str(self.get_parameter("interface").value)
        self.arm_velocity_limit = float(self.get_parameter("arm_velocity_limit").value)
        self.rate_hz      = float(self.get_parameter("rate_hz").value)
        self.frame        = str(self.get_parameter("ik_world_frame").value)
        self.ik_alpha     = float(self.get_parameter("ik_alpha").value)
        self.ik_goal_filter_alpha = float(self.get_parameter("ik_goal_filter_alpha").value)
        self.ik_orientation_mode  = str(self.get_parameter("ik_orientation_mode").value).lower()
        self.ik_max_ori_step_rad  = float(self.get_parameter("ik_max_ori_step_rad").value)
        self.ee_auto_calibrate    = bool(self.get_parameter("ee_auto_calibrate").value)
        self.auto_reissue_goals   = bool(self.get_parameter("auto_reissue_goals").value)
        self.goal_pos_tol         = float(self.get_parameter("goal_pos_tol").value)
        self.goal_ori_tol_deg     = float(self.get_parameter("goal_ori_tol_deg").value)
        stop_nav_topic            = str(self.get_parameter("stop_nav_topic").value)
        self.release_on_disable   = bool(self.get_parameter("release_on_disable").value)

        def _pvec(name):
            return np.array(self.get_parameter(name).value, dtype=float)

        self._ee_off_right_xyz     = _pvec("ee_offset_right_xyz")
        self._ee_off_right_rpy_deg = _pvec("ee_offset_right_rpy_deg")
        self._ee_off_left_xyz      = _pvec("ee_offset_left_xyz")
        self._ee_off_left_rpy_deg  = _pvec("ee_offset_left_rpy_deg")

        # ----- internal state -----
        self.motor_state = [MotorState() for _ in range(35)]
        self.lowstate_buffer = DataBuffer()
        self._last_q_target = np.zeros(14, dtype=float)
        self.arms_enabled = False
        self.homing_active = False
        self.homing_reached = False
        self.homing_tolerance = 0.02
        self._last_left_goal_raw = None
        self._last_right_goal_raw = None
        self._goal_left_filt = None
        self._goal_right_filt = None
        self._reset_after_home = False
        self._initialized = False

        self._T_off_right_static = self._mk_static_T(self._ee_off_right_xyz, self._ee_off_right_rpy_deg)
        self._T_off_left_static  = self._mk_static_T(self._ee_off_left_xyz, self._ee_off_left_rpy_deg)
        self._T_off_right_auto = None
        self._T_off_left_auto  = None
        self._auto_done_right  = False
        self._auto_done_left   = False

        # ----- IK solver -----
        self.ik_solver = G1IKSolver(debug=False)
        if hasattr(self.ik_solver, "set_orientation_mode"):
            self.ik_solver.set_orientation_mode(self.ik_orientation_mode)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Home joint angles (from g1pilot config)
        self.home_right = np.array([0.6604, -0.0925, 0.0222, -0.8391, 0.1072, 0.2716, 0.0674], dtype=float)
        self.home_left  = np.array([0.9230, -0.0600, 0.0373, -0.7793, -0.0661, -0.1140, -0.2975], dtype=float)

        # ----- ROS publishers -----
        # BotBrain namespace convention: /{robot_name}/manipulation/...
        self.left_workspace_publisher  = self.create_publisher(Marker, 'manipulation/workspace/left', 10)
        self.right_workspace_publisher = self.create_publisher(Marker, 'manipulation/workspace/right', 10)

        # ★ Mode-B: zero-velocity publisher for twist_mux → stops base when arms active
        self._stop_nav_pub = self.create_publisher(Twist, stop_nav_topic, 10)
        self._stop_nav_timer = None  # created when arms are enabled

        if not self.use_robot:
            self.joint_pub = self.create_publisher(JointState, "joint_states", 10)

        # ----- ROS subscriptions  (BotBrain namespace) -----
        self.create_subscription(PoseStamped, "manipulation/hand_goal/right", self._right_goal_callback, 10)
        self.create_subscription(PoseStamped, "manipulation/hand_goal/left",  self._left_goal_callback, 10)
        self.create_subscription(Bool, "manipulation/enabled", self._arms_controlled_callback, 10)
        self.create_subscription(Bool, "manipulation/home",    self._homming_callback, 10)

        # ----- DDS init -----
        if self.use_robot:
            self._init_robot_interface()

        self._last_tick_time = None
        self.timer = self.create_timer(1.0 / self.rate_hz, self.main_loop)

    # ---------------------------------------------------------------
    # BotBrain config helpers
    # ---------------------------------------------------------------
    def _read_botbrain_config(self):
        """Read robot_config.yaml to get robot_name and network_interface."""
        cfg_path = "/botbrain_ws/robot_config.yaml"
        if not os.path.isfile(cfg_path):
            # Fallback for local dev
            cfg_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "../../../../robot_config.yaml"
            )
        try:
            with open(cfg_path, 'r') as f:
                cfg = yaml.safe_load(f).get("robot_configuration", {})
            self._robot_name = cfg.get("robot_name", "g1_robot")
            self._network_interface = cfg.get("network_interface", "eth0")
        except Exception:
            self._robot_name = "g1_robot"
            self._network_interface = "eth0"
        self.get_logger().info(f"BotBrain config: robot_name={self._robot_name}, iface={self._network_interface}")

    # ---------------------------------------------------------------
    # Mode-B helpers — stop nav while arms active
    # ---------------------------------------------------------------
    def _start_stop_nav(self):
        """Create a 10 Hz timer that publishes zero Twist on manipulation_vel."""
        if self._stop_nav_timer is not None:
            return
        self._stop_nav_timer = self.create_timer(0.1, self._pub_zero_vel)
        self.get_logger().info("[Mode-B] Navigation STOPPED — arm control active.")

    def _stop_stop_nav(self):
        """Destroy the zero-velocity timer → twist_mux stops locking, nav resumes."""
        if self._stop_nav_timer is not None:
            self._stop_nav_timer.cancel()
            self.destroy_timer(self._stop_nav_timer)
            self._stop_nav_timer = None
            self.get_logger().info("[Mode-B] Navigation RESUMED — arm control disabled.")

    def _release_arm_control(self):
        """渐进式交还上身控制权给运控（对齐 g1_driver.cpp release_arm_joints）。

        通过 kNotUsedJoint0.q (weight 关节) 在 2 秒内从 1.0 线性降至 0.0，
        运控按 (1-weight) 比例逐渐接管手臂，实现平滑过渡。
        最后发 mode_pr=0 彻底释放 arm_sdk 通道。
        """
        if not self.use_robot or not getattr(self, "_initialized", False):
            return

        try:
            current_all = self.get_current_motor_q()
        except Exception:
            current_all = None

        # ── Phase 1: weight 从 1.0 渐降到 0.0（2 秒，50 Hz，100 步）──
        duration_sec = 2.0
        dt = 0.02   # 20 ms
        steps = int(duration_sec / dt)  # 100

        self.get_logger().info("[Release] Ramping arm weight down over 2s...")

        for i in range(steps + 1):
            weight = 1.0 - float(i) / float(steps)  # 1.0 → 0.0

            self.msg.mode_pr = 1
            self.msg.mode_machine = self.get_mode_machine()

            # weight 关节 (index 29) — 控制 arm_sdk 混合权重
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].mode = 1
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q    = weight
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].dq   = 0.0
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].kp   = 0.0
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].kd   = 0.0

            # 手臂关节保持当前位置 + 正常增益（有力矩）
            wrist_vals = {m.value for m in G1_29_JointWristIndex}
            for jid in G1_29_JointArmIndex:
                self.msg.motor_cmd[jid].mode = 1
                if current_all is not None:
                    self.msg.motor_cmd[jid].q = float(current_all[jid.value])
                self.msg.motor_cmd[jid].dq  = 0.0
                self.msg.motor_cmd[jid].tau = 0.0
                if jid.value in wrist_vals:
                    self.msg.motor_cmd[jid].kp = self.kp_wrist
                    self.msg.motor_cmd[jid].kd = self.kd_wrist
                else:
                    self.msg.motor_cmd[jid].kp = self.kp_low
                    self.msg.motor_cmd[jid].kd = self.kd_low

            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)
            time.sleep(dt)

        # ── Phase 2: weight=0 后发 mode_pr=0 彻底释放 ──
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()
        for _ in range(5):
            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)
            time.sleep(0.002)

        self.get_logger().info("[Release] Arm control released — upper body returned to motion controller.")

    def _hold_arm_position(self):
        """禁用后保持手臂当前位置（不交还控制权）。

        仍通过 arm_sdk (mode_pr=1) 控制，以当前关节角为目标，
        使用较低增益做阻尼保持。
        """
        if not self.use_robot or not getattr(self, "_initialized", False):
            return

        try:
            current_all = self.get_current_motor_q()
        except Exception:
            return

        self.msg.mode_pr = 1
        self.msg.mode_machine = self.get_mode_machine()

        wrist_vals = {m.value for m in G1_29_JointWristIndex}
        for jid in G1_29_JointArmIndex:
            self.msg.motor_cmd[jid].mode = 1
            self.msg.motor_cmd[jid].q    = float(current_all[jid.value])
            self.msg.motor_cmd[jid].dq   = 0.0
            self.msg.motor_cmd[jid].tau  = 0.0
            if jid.value in wrist_vals:
                self.msg.motor_cmd[jid].kp = self.kp_wrist
                self.msg.motor_cmd[jid].kd = self.kd_wrist
            else:
                self.msg.motor_cmd[jid].kp = self.kp_low
                self.msg.motor_cmd[jid].kd = self.kd_low

        self.msg.crc = self.crc.Crc(self.msg)
        self.lowcmd_publisher.Write(self.msg)

    def _pub_zero_vel(self):
        self._stop_nav_pub.publish(Twist())  # all zeros

    # ---------------------------------------------------------------
    # SE3 / IK helpers  (same logic as g1pilot)
    # ---------------------------------------------------------------
    def _mk_static_T(self, xyz, rpy_deg):
        rpy = np.radians(np.array(rpy_deg, dtype=float))
        R = pin.rpy.rpyToMatrix(rpy[0], rpy[1], rpy[2])
        return SE3(R, np.array(xyz, dtype=float))

    def _goal_error(self, side: str, T_goal: SE3):
        M_cur = self._fk_current_ee(side)
        if M_cur is None or T_goal is None:
            return None, None
        dp = float(np.linalg.norm(T_goal.translation - M_cur.translation))
        dq = pin.Quaternion(M_cur.rotation.T @ T_goal.rotation)
        ang = 2.0 * math.atan2(
            math.sqrt(dq.x * dq.x + dq.y * dq.y + dq.z * dq.z),
            abs(dq.w)
        )
        return dp, ang

    def _lowpass_goal(self, T_prev: SE3, T_new: SE3, alpha: float) -> SE3:
        if T_prev is None:
            return T_new
        p = (1.0 - alpha) * T_prev.translation + alpha * T_new.translation
        q0 = _mat_to_quat_wxyz(T_prev.rotation)
        q1 = _mat_to_quat_wxyz(T_new.rotation)
        qf = (1 - alpha) * q0 + alpha * q1
        qf = qf / np.linalg.norm(qf)
        Rf = _quat_wxyz_to_matrix(qf)
        return SE3(Rf, p)

    def _limit_ori_step(self, R_cur, R_des, max_step):
        R_err = R_cur.T @ R_des
        aa = pin.log3(R_err)
        nrm = float(np.linalg.norm(aa))
        if nrm <= 1e-12 or nrm <= max_step:
            return R_des
        aa_lim = aa * (max_step / nrm)
        return R_cur @ pin.exp3(aa_lim)

    def _fk_current_ee(self, side: str):
        try:
            q_full = pin.neutral(self.ik_solver.model)
            cur_all = self.get_current_motor_q() if self.use_robot else self._assemble_full_from_last()
            for jid_idx, ros_name in enumerate(self.ik_solver._ros_joint_names):
                if ros_name in self.ik_solver._name_to_q_index:
                    q_full[self.ik_solver._name_to_q_index[ros_name]] = float(cur_all[jid_idx])
            pin.forwardKinematics(self.ik_solver.model, self.ik_solver.data, q_full)
            pin.updateFramePlacements(self.ik_solver.model, self.ik_solver.data)
            fid = self.ik_solver._fid_right if side == 'right' else self.ik_solver._fid_left
            return self.ik_solver.data.oMf[fid]
        except Exception:
            return None

    def _gate_auto_calibration(self, T_goal_in, side):
        M_cur = self._fk_current_ee(side)
        if M_cur is None:
            return None
        dp = np.linalg.norm(T_goal_in.translation - M_cur.translation)
        dq = pin.Quaternion(M_cur.rotation.T @ T_goal_in.rotation)
        ang = 2 * math.atan2(np.linalg.norm([dq.x, dq.y, dq.z]), abs(dq.w))
        if dp < 0.05 and ang < math.radians(12.0):
            return M_cur
        return None

    def _apply_offsets_and_filters(self, side: str, T_goal_input: SE3):
        T_static = self._T_off_right_static if side == 'right' else self._T_off_left_static
        T_auto = self._T_off_right_auto if side == 'right' else self._T_off_left_auto
        auto_done = self._auto_done_right if side == 'right' else self._auto_done_left

        if self.ee_auto_calibrate and not auto_done:
            M_cur_ok = self._gate_auto_calibration(T_goal_input, side)
            if M_cur_ok is not None:
                T_pre = T_goal_input * T_static
                T_auto_new = T_pre.inverse() * M_cur_ok
                if side == 'right':
                    self._T_off_right_auto = T_auto_new
                    self._auto_done_right = True
                else:
                    self._T_off_left_auto = T_auto_new
                    self._auto_done_left = True
                T_auto = T_auto_new

        T_raw = T_goal_input * T_static * (T_auto if T_auto is not None else SE3.Identity())

        if side == 'right':
            self._goal_right_filt = self._lowpass_goal(self._goal_right_filt, T_raw, self.ik_goal_filter_alpha)
            T_use = self._goal_right_filt
        else:
            self._goal_left_filt = self._lowpass_goal(self._goal_left_filt, T_raw, self.ik_goal_filter_alpha)
            T_use = self._goal_left_filt

        M_cur = self._fk_current_ee(side)
        if M_cur is not None and T_use is not None:
            R_lim = self._limit_ori_step(M_cur.rotation, T_use.rotation, self.ik_max_ori_step_rad)
            T_use = SE3(R_lim, T_use.translation.copy())

        return T_use

    # ---------------------------------------------------------------
    # DDS init
    # ---------------------------------------------------------------
    def _init_robot_interface(self):
        ChannelFactoryInitialize(0, self.interface)

        self.lowstate_subscriber = ChannelSubscriber('rt/lowstate', LowState_)
        self.lowstate_subscriber.Init()

        self.subscribe_thread = threading.Thread(target=self._subscribe_motor_state, daemon=True)
        self.subscribe_thread.start()

        self.lowcmd_publisher = ChannelPublisher('rt/arm_sdk', LowCmd_)
        self.lowcmd_publisher.Init()

        while not self.lowstate_buffer.GetData():
            self.get_logger().info("Waiting for LowState data...")
            time.sleep(0.01)

        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()

        self.kp_high  = 300.0; self.kd_high  = 3.0
        self.kp_low   = 150.0; self.kd_low   = 4.0
        self.kp_wrist = 40.0;  self.kd_wrist = 1.5

        wrist_vals = {m.value for m in G1_29_JointWristIndex}
        for jid in G1_29_JointArmIndex:
            self.msg.motor_cmd[jid].mode = 1
            if jid.value in wrist_vals:
                self.msg.motor_cmd[jid].kp = self.kp_wrist
                self.msg.motor_cmd[jid].kd = self.kd_wrist
            else:
                self.msg.motor_cmd[jid].kp = self.kp_low
                self.msg.motor_cmd[jid].kd = self.kd_low
            self.msg.motor_cmd[jid].q = float(self.get_current_motor_q()[jid.value])

        self.q_target = np.zeros(14)
        self.tauff_target = np.zeros(14)
        self._initialized = True

    def _subscribe_motor_state(self):
        while rclpy.ok():
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                self.lowstate_buffer.SetData(msg)
                for i in range(len(self.motor_state)):
                    self.motor_state[i].q  = msg.motor_state[i].q
                    self.motor_state[i].dq = msg.motor_state[i].dq
            time.sleep(0.001)

    def get_mode_machine(self) -> int:
        msg = self.lowstate_buffer.GetData()
        return getattr(msg, "mode_machine", 0) if msg is not None else 0

    def get_current_motor_q(self) -> np.ndarray:
        msg = self.lowstate_buffer.GetData()
        return np.array([msg.motor_state[id].q for id in G1_29_JointIndex], dtype=float)

    def _assemble_full_from_last(self) -> np.ndarray:
        full = np.zeros(29, dtype=float)
        for i, jidx in enumerate(LEFT_JOINT_INDICES_LIST):
            full[jidx] = self._last_q_target[i]
        for i, jidx in enumerate(RIGHT_JOINT_INDICES_LIST):
            full[jidx] = self._last_q_target[7 + i]
        return full

    def _hold_non_arm_joints(self):
        if not self.use_robot:
            return
        arm_vals  = {m.value for m in G1_29_JointArmIndex}
        weak_vals = {m.value for m in G1_29_JointWeakIndex}
        current_all = self.get_current_motor_q()

        self.msg.mode_pr = 0
        for jid in G1_29_JointIndex:
            if jid.value in arm_vals:
                continue
            self.msg.motor_cmd[jid].mode = 1
            if jid.value in weak_vals:
                self.msg.motor_cmd[jid].kp = self.kp_low
                self.msg.motor_cmd[jid].kd = self.kd_low
            else:
                self.msg.motor_cmd[jid].kp = self.kp_high
                self.msg.motor_cmd[jid].kd = self.kd_high
            self.msg.motor_cmd[jid].q   = float(current_all[jid.value])
            self.msg.motor_cmd[jid].dq  = 0.0
            self.msg.motor_cmd[jid].tau = 0.0

    # ---------------------------------------------------------------
    # ROS callbacks
    # ---------------------------------------------------------------
    def _arms_controlled_callback(self, msg: Bool):
        self.arms_enabled = msg.data
        if self.arms_enabled:
            try:
                cur = self.get_current_motor_q()
                left  = [cur[j] for j in LEFT_JOINT_INDICES_LIST]
                right = [cur[j] for j in RIGHT_JOINT_INDICES_LIST]
                self._last_q_target = np.array(left + right, dtype=float)
            except Exception:
                pass
            self._start_stop_nav()   # ★ Mode-B: stop base
            self.get_logger().info("Arm ENABLED (Mode-B: base stopped).")
        else:
            # 运行时可通过 ros2 param set 动态修改
            self.release_on_disable = bool(self.get_parameter("release_on_disable").value)
            if self.release_on_disable:
                self._release_arm_control()  # ★ 交还上身控制权
                self.get_logger().info("Arm DISABLED — control RELEASED to motion controller.")
            else:
                self._hold_arm_position()    # ★ 保持当前位置
                self.get_logger().info("Arm DISABLED — HOLDING current position (release_on_disable=false).")
            self._stop_stop_nav()            # ★ Mode-B: resume nav

    def _homming_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Moving both arms to HOME position.")
            self.homing_active  = True
            self.homing_reached = False
            self._reset_after_home = False
            if hasattr(self.ik_solver, "clear_goals"):
                self.ik_solver.clear_goals()

    def _transform_pose_to_world(self, ps: PoseStamped) -> PoseStamped:
        if not ps.header.frame_id or ps.header.frame_id == self.frame:
            return ps
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame, ps.header.frame_id, Time(), timeout=Duration(seconds=0.2)
            )
            return do_transform_pose(ps, tf)
        except Exception as e:
            self.get_logger().warning(f"[IK] TF {ps.header.frame_id}->{self.frame} failed: {e}")
            return ps

    def _process_goal(self, msg: PoseStamped, side: str):
        """Shared goal processing for left/right callbacks."""
        if self.homing_active or not self.arms_enabled:
            return

        if self._reset_after_home:
            self._reset_after_home = False
            self.homing_reached = False
            try:
                cur = self.get_current_motor_q()
                left  = [cur[j] for j in LEFT_JOINT_INDICES_LIST]
                right = [cur[j] for j in RIGHT_JOINT_INDICES_LIST]
                self._last_q_target = np.array(left + right, dtype=float)
            except Exception:
                self._last_q_target = np.concatenate((self.home_left, self.home_right)).copy()

            self._goal_left_filt  = None
            self._goal_right_filt = None
            self.ik_solver.set_current_configuration({
                "left":  self._last_q_target[0:7].copy(),
                "right": self._last_q_target[7:14].copy()
            })

        msg_tf = self._transform_pose_to_world(msg)
        o, p = msg_tf.pose.orientation, msg_tf.pose.position
        q = pin.Quaternion(o.w, o.x, o.y, o.z)
        T_goal_in = SE3(q.matrix(), np.array([p.x, p.y, p.z]))

        if side == 'right':
            self._last_right_goal_raw = T_goal_in
        else:
            self._last_left_goal_raw = T_goal_in

        T_goal_use = self._apply_offsets_and_filters(side, T_goal_in)
        if T_goal_use is not None:
            self.ik_solver.set_goal(side, T_goal_use)

    def _right_goal_callback(self, msg: PoseStamped):
        self._process_goal(msg, 'right')

    def _left_goal_callback(self, msg: PoseStamped):
        self._process_goal(msg, 'left')

    # ---------------------------------------------------------------
    # Workspace visualisation
    # ---------------------------------------------------------------
    def _publish_workspace(self, arm):
        marker = Marker()
        marker.header.frame_id = WORKSPACE["frame"]
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "workspace"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.005
        marker.color = ColorRGBA(r=0.1, g=1.0, b=0.3, a=0.9)

        points = WORKSPACE[arm]
        pts = {k: Point(x=v[0], y=v[1], z=v[2]) for k, v in points.items()}

        edges = [
            ("left_bottom_front",  "right_bottom_front"),
            ("right_bottom_front", "right_bottom_back"),
            ("right_bottom_back",  "left_bottom_back"),
            ("left_bottom_back",   "left_bottom_front"),
            ("left_top_front",     "right_top_front"),
            ("right_top_front",    "right_top_back"),
            ("right_top_back",     "left_top_back"),
            ("left_top_back",      "left_top_front"),
            ("left_bottom_front",  "left_top_front"),
            ("right_bottom_front", "right_top_front"),
            ("left_bottom_back",   "left_top_back"),
            ("right_bottom_back",  "right_top_back"),
        ]

        for a, b in edges:
            marker.points.append(pts[a])
            marker.points.append(pts[b])

        if arm == "left_arm":
            self.left_workspace_publisher.publish(marker)
        else:
            self.right_workspace_publisher.publish(marker)

    # ---------------------------------------------------------------
    # Main loop (250 Hz)
    # ---------------------------------------------------------------
    def _compute_dt(self) -> float:
        now = time.time()
        if self._last_tick_time is None:
            dt = 1.0 / self.rate_hz
        else:
            dt = max(1e-4, min(0.1, now - self._last_tick_time))
        self._last_tick_time = now
        return dt

    def main_loop(self):
        self._publish_workspace("left_arm")
        self._publish_workspace("right_arm")

        if not getattr(self, "_initialized", False):
            return

        if self.use_robot:
            robot_data = self.lowstate_subscriber.Read()
            if robot_data is not None:
                self.lowstate_buffer.SetData(robot_data)
                for i in range(len(self.motor_state)):
                    self.motor_state[i].q  = robot_data.motor_state[i].q
                    self.motor_state[i].dq = robot_data.motor_state[i].dq

        if not self.arms_enabled:
            self._hold_non_arm_joints()
            # release_on_disable=false 时，持续发 mode_pr=1 保持手臂位置
            if not self.release_on_disable and self.use_robot:
                self._hold_arm_position()
            return

        # --- homing sequence ---
        if self.homing_active:
            q_target = np.concatenate((self.home_left, self.home_right))
            if np.linalg.norm(q_target - self._last_q_target) < self.homing_tolerance:
                self.homing_active  = False
                self.homing_reached = True
                self._last_q_target = q_target.copy()

                if hasattr(self.ik_solver, "clear_goals"):
                    self.ik_solver.clear_goals()

                self.ik_solver.set_current_configuration({
                    "left":  self.home_left.copy(),
                    "right": self.home_right.copy()
                })

                try:
                    q_full = pin.neutral(self.ik_solver.model)
                    for i, arm_i in enumerate(LEFT_JOINT_INDICES_LIST):
                        q_full[self.ik_solver._name_to_q_index[self.ik_solver._ros_joint_names[arm_i]]] = self.home_left[i]
                    for i, arm_i in enumerate(RIGHT_JOINT_INDICES_LIST):
                        q_full[self.ik_solver._name_to_q_index[self.ik_solver._ros_joint_names[arm_i]]] = self.home_right[i]

                    pin.forwardKinematics(self.ik_solver.model, self.ik_solver.data, q_full)
                    pin.updateFramePlacements(self.ik_solver.model, self.ik_solver.data)

                    T_left  = self.ik_solver.data.oMf[self.ik_solver._fid_left]
                    T_right = self.ik_solver.data.oMf[self.ik_solver._fid_right]

                    self._goal_left_filt  = T_left.copy()
                    self._goal_right_filt = T_right.copy()
                    if hasattr(self.ik_solver, "set_goal"):
                        self.ik_solver.set_goal("left",  T_left.copy())
                        self.ik_solver.set_goal("right", T_right.copy())

                    self._reset_after_home = True
                except Exception as e:
                    self.get_logger().warning(f"Failed to align IK goals with home: {e}")

                self.get_logger().info("Home position reached.")

        elif self.homing_reached:
            q_target = np.concatenate((self.home_left, self.home_right))

        else:
            # --- normal IK tracking ---
            current_all = self.get_current_motor_q() if self.use_robot else self._assemble_full_from_last()

            try:
                self.ik_solver.set_current_configuration({
                    "left":  self._last_q_target[0:7].copy(),
                    "right": self._last_q_target[7:14].copy()
                })
            except Exception:
                pass

            if self._goal_left_filt is not None:
                self.ik_solver.set_goal("left", self._goal_left_filt)
            if self._goal_right_filt is not None:
                self.ik_solver.set_goal("right", self._goal_right_filt)

            q_dict = self.ik_solver.get_joint_targets(current_all)

            # ★ SAFETY: if no IK goals are set, hold current position
            #   (prevents arms drifting to zero when enabled with no goal)
            q_target = self._last_q_target.copy()
            if "left" in q_dict:
                q_target[0:7] = q_dict["left"]
            if "right" in q_dict:
                q_target[7:14] = q_dict["right"]

        # --- velocity limit + smoothing ---
        dt = self._compute_dt()
        max_step = self.arm_velocity_limit * dt
        dq = np.clip(q_target - self._last_q_target, -max_step, max_step)

        q_unsmoothed = self._last_q_target + dq
        q_smooth = (1.0 - self.ik_alpha) * self._last_q_target + self.ik_alpha * q_unsmoothed
        self._last_q_target = q_smooth.copy()

        # --- publish ---
        if self.use_robot:
            self.msg.mode_machine = self.get_mode_machine()
            self.msg.mode_pr = 1

            try:
                self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = 1.0
            except Exception:
                pass

            wrist_vals = {m.value for m in G1_29_JointWristIndex}
            for idx, jid in enumerate(G1_29_JointArmIndex):
                self.msg.motor_cmd[jid].mode = 1
                self.msg.motor_cmd[jid].q    = float(q_smooth[idx])
                self.msg.motor_cmd[jid].dq   = 0.0
                self.msg.motor_cmd[jid].tau  = float(0.0)
                if jid.value in wrist_vals:
                    self.msg.motor_cmd[jid].kp = self.kp_wrist
                    self.msg.motor_cmd[jid].kd = self.kd_wrist
                else:
                    self.msg.motor_cmd[jid].kp = self.kp_low
                    self.msg.motor_cmd[jid].kd = self.kd_low

            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)
        else:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [JOINT_NAMES_ROS[i] for i in sorted(JOINT_NAMES_ROS.keys())]
            js.position = [0.0] * len(js.name)
            for idx, joint_idx in enumerate(LEFT_JOINT_INDICES_LIST):
                js.position[joint_idx] = float(q_smooth[idx])
            for idx, joint_idx in enumerate(RIGHT_JOINT_INDICES_LIST):
                js.position[joint_idx] = float(q_smooth[7 + idx])
            self.joint_pub.publish(js)


def _read_interface_from_yaml() -> str:
    """Read network_interface from robot_config.yaml (before ROS node exists)."""
    cfg_path = "/botbrain_ws/robot_config.yaml"
    try:
        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f).get("robot_configuration", {})
        return cfg.get("network_interface", "eth0")
    except Exception:
        return "eth0"


def main(args=None):
    # ── 防止 DDS domain 冲突 (monkey-patch) ──
    # pip cyclonedds 和 rmw_cyclonedds_cpp 共用同一个 libddsc.so。
    # rmw_cyclonedds_cpp 在 rmw_create_node 时**总会**调用
    #   dds_create_domain(0, config)  （config 含 CYCLONEDDS_URI）
    # unitree_sdk2py 的 ChannelFactoryInitialize 也会调用
    #   cyclonedds.domain.Domain(0, xml) → dds_create_domain(0, xml)
    # CycloneDDS C 库不允许同一进程对同一 domain-id 调两次 dds_create_domain
    # → "Precondition Not Met"。
    #
    # 解决：用空壳 Domain 替换 cyclonedds.domain.Domain，使
    # ChannelFactoryInitialize 记录 __initialized=True 但不真正建域。
    # domain 0 统一由 rmw 通过 CYCLONEDDS_URI (start_manipulation.sh 设置，
    # 含正确网口) 创建。之后 unitree SDK 的 DomainParticipant(domain_id=0)
    # 会加入 rmw 已建好的 domain 0。
    # ── monkey-patch：替换 ChannelFactory 看到的 Domain ──
    # channel.py 用 `from cyclonedds.domain import Domain` 做了本地绑定，
    # 所以必须 patch unitree_sdk2py.core.channel 模块内的 Domain，
    # 而非 cyclonedds.domain.Domain（那个改了 channel.py 也看不到）。
    import unitree_sdk2py.core.channel as _ch_mod

    class _NoOpDomain:
        """Dummy Domain — 跳过 dds_create_domain。"""
        def __init__(self, *a, **kw):
            pass
        def close(self):
            pass
        def __del__(self):
            pass

    _ch_mod.Domain = _NoOpDomain

    # ChannelFactoryInitialize 已在 ArmController.__init__ → _init_robot_interface()
    # 中调用（在 super().__init__ 之后），此时 rmw 已建好 domain 0，
    # DomainParticipant(0) 会加入已有 domain 而非隐式新建。

    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
