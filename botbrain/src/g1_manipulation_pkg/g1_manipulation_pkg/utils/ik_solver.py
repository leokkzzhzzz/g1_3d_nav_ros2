#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pinocchio-based IK solver for Unitree G1-29 arms.
Migrated from g1pilot to BotBrain g1_manipulation_pkg.
"""

import os
import math
import numpy as np
import pinocchio as pin
from pinocchio import SE3
from ament_index_python.packages import get_package_share_directory

from g1_manipulation_pkg.utils.joints_names import (
    JOINT_NAMES_ROS,
    JOINT_LIMITS_RAD,
    RIGHT_JOINT_INDICES_LIST,
    LEFT_JOINT_INDICES_LIST,
)
from g1_manipulation_pkg.utils.helpers import (
    clamp, wrap_to_pi,
    mat_to_quat_wxyz, quat_wxyz_to_matrix,
    quat_slerp, yaw_from_R,
)


class G1IKSolver:
    """
    Inverse kinematics solver for Unitree G1-29 arms using Pinocchio,
    with smooth orientation control, damping, and optional collision avoidance.
    """

    def __init__(self,
                 urdf_path=None,
                 mesh_dir=None,
                 world_frame='pelvis',
                 frame_left='left_hand_point_contact',
                 frame_right='right_hand_point_contact',
                 alpha=0.2,
                 max_dq_step=0.05,
                 damping=1e-6,
                 max_iter=60,
                 tol=1e-4,
                 pos_gain=1.0,
                 ori_gain=0.8,
                 adaptive_damping=True,
                 sigma_min_thresh=0.08,
                 lambda_base=1e-6,
                 lambda_max=1e-1,
                 max_ori_step_rad=0.35,
                 goal_filter_alpha=0.25,
                 orientation_mode="full",
                 debug=False,
                 enable_collision_avoidance=True,
                 collision_distance_thresh=0.05,
                 collision_gain=10.0):
        self.alpha = alpha
        self.max_dq_step = max_dq_step
        self.damping = damping
        self.max_iter = max_iter
        self.tol = tol
        self.pos_gain = pos_gain
        self.ori_gain = ori_gain
        self.adaptive_damping = adaptive_damping
        self.sigma_min_thresh = sigma_min_thresh
        self.lambda_base = lambda_base
        self.lambda_max = lambda_max
        self.max_ori_step_rad = max_ori_step_rad
        self.goal_filter_alpha = goal_filter_alpha
        self.orientation_mode = orientation_mode
        self.debug = debug
        self.world_frame = world_frame
        self.frame_left = frame_left
        self.frame_right = frame_right

        # Collision avoidance parameters
        self.enable_collision_avoidance = enable_collision_avoidance
        self.collision_distance_thresh = collision_distance_thresh
        self.collision_gain = collision_gain

        # Load model
        try:
            if urdf_path is None or mesh_dir is None:
                # Use g1_manipulation_pkg share directory
                pkg_share = get_package_share_directory('g1_manipulation_pkg')
                urdf_path = urdf_path or os.path.join(pkg_share, 'description_files', 'urdf', '29dof.urdf')
                # package_dirs must point to parent of "g1_manipulation_pkg" so that
                # package://g1_manipulation_pkg/description_files/meshes/xxx.STL resolves
                mesh_dir = mesh_dir or os.path.dirname(pkg_share)

            print(f"Loading URDF model for IK solver...\n  urdf: {urdf_path}\n  pkg_dir: {mesh_dir}", flush=True)
            try:
                self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(
                    urdf_path,
                    package_dirs=[mesh_dir],
                    geometry_types=[pin.GeometryType.COLLISION, pin.GeometryType.VISUAL]
                )
            except Exception as e:
                print(f"Failed to load collision model properly: {e}", flush=True)
                print("→ Retrying with collision-only model...", flush=True)
                self.model, self.collision_model = pin.buildModelsFromUrdf(
                    urdf_path,
                    package_dirs=[mesh_dir],
                    geometry_types=[pin.GeometryType.COLLISION]
                )
                self.visual_model = None

            self.data = pin.Data(self.model)
            self.collision_data = pin.GeometryData(self.collision_model)

        except Exception as e:
            raise RuntimeError(f"Failed to load URDF: {e}")

        # Joint mapping
        self._ros_joint_names = [JOINT_NAMES_ROS[i] for i in range(29)]
        self._name_to_q_index = {}
        self._name_to_v_index = {}
        for j in range(1, self.model.njoints):
            jnt = self.model.joints[j]
            if jnt.nq == 1:
                nm = self.model.names[j]
                if nm in self._ros_joint_names:
                    self._name_to_q_index[nm] = jnt.idx_q
                    self._name_to_v_index[nm] = jnt.idx_v

        # Frame IDs
        self._fid_right = self.model.getFrameId(frame_right)
        self._fid_left  = self.model.getFrameId(frame_left)

        # State buffers
        self._goal_right = None
        self._goal_left  = None
        self._prev_q_full = None
        self._prev_q14 = None

    # --------------------------------------------------------
    # Helper functions
    # --------------------------------------------------------

    def _limit_ori_step(self, R_cur, R_des):
        R_err = R_cur.T @ R_des
        aa = pin.log3(R_err)
        norm = np.linalg.norm(aa)
        if norm < 1e-12 or norm <= self.max_ori_step_rad:
            return R_des
        aa_limited = aa * (self.max_ori_step_rad / norm)
        return R_cur @ pin.exp3(aa_limited)

    def _lowpass_goal(self, T_prev, T_new):
        if T_prev is None:
            return T_new
        p = (1 - self.goal_filter_alpha) * T_prev.translation + self.goal_filter_alpha * T_new.translation
        q0 = mat_to_quat_wxyz(T_prev.rotation)
        q1 = mat_to_quat_wxyz(T_new.rotation)
        qf = quat_slerp(q0, q1, self.goal_filter_alpha)
        return SE3(quat_wxyz_to_matrix(qf), p)

    def _collision_repulsion(self, q):
        """Compute joint-space repulsion based on proximity collisions."""
        if not self.enable_collision_avoidance:
            return np.zeros(self.model.nv)

        pin.updateGeometryPlacements(self.model, self.data, self.collision_model, self.collision_data)
        pin.computeCollisions(self.model, self.data, self.collision_model, self.collision_data, q, False)

        dq_repulse = np.zeros(self.model.nv)
        total_weight = 0.0

        for res in self.collision_data.collisionResults:
            if not res.isCollision():
                continue

            d = res.distance
            if d <= 1e-1 or d > self.collision_distance_thresh:
                continue

            weight = math.exp(-4.0 * d / self.collision_distance_thresh)
            n = np.array(res.normal)
            f_repulse = self.collision_gain * weight * (self.collision_distance_thresh - d) * n

            geom_id = res.firstGeomIdx
            geom = self.collision_model.geometryObjects[geom_id]
            parent_joint = geom.parentJoint

            J = pin.computeJointJacobian(self.model, self.data, q, parent_joint)
            J_norm = np.linalg.norm(J)
            if J_norm < 1e-8:
                continue

            dq_local = (J.T @ np.concatenate([f_repulse, np.zeros(3)])) / J_norm
            dq_repulse += dq_local
            total_weight += 1.0

        if total_weight > 0:
            dq_repulse /= total_weight

        dq_repulse = np.clip(dq_repulse, -0.05, 0.05)
        return dq_repulse

    # --------------------------------------------------------
    # Public API
    # --------------------------------------------------------

    def clear_goals(self):
        """Clear all pending IK goals."""
        self._goal_right = None
        self._goal_left = None

    def set_current_configuration(self, q_dict: dict):
        """Manually set the current joint configuration for the solver."""
        q_full = pin.neutral(self.model)

        if 'left' in q_dict:
            for i, arm_i in enumerate(LEFT_JOINT_INDICES_LIST):
                q_full[self._name_to_q_index[self._ros_joint_names[arm_i]]] = q_dict['left'][i]

        if 'right' in q_dict:
            for i, arm_i in enumerate(RIGHT_JOINT_INDICES_LIST):
                q_full[self._name_to_q_index[self._ros_joint_names[arm_i]]] = q_dict['right'][i]

        self._prev_q_full = q_full.copy()
        self._prev_q14 = np.concatenate([
            q_dict.get('left', np.zeros(7)),
            q_dict.get('right', np.zeros(7))
        ])

    def set_goal(self, side, T_goal: SE3):
        """Set a new SE3 target for the given side ('left' or 'right')."""
        if side == "right":
            self._goal_right = self._lowpass_goal(self._goal_right, T_goal)
        elif side == "left":
            self._goal_left = self._lowpass_goal(self._goal_left, T_goal)

    def get_joint_targets(self, current_all: np.ndarray, q_init: np.ndarray = None):
        """Compute current joint targets given current joint positions."""
        out = {}
        if self._goal_left is not None:
            q_left = self.solve('left', q_init, current_all)
            if q_left is not None:
                out['left'] = q_left
        if self._goal_right is not None:
            q_right = self.solve('right', q_init, current_all)
            if q_right is not None:
                out['right'] = q_right
        return out

    def solve(self, side: str, q_init: np.ndarray, current_all: np.ndarray) -> np.ndarray:
        """Compute 7-DOF IK solution for one arm (collision-aware)."""
        fid = self._fid_right if side == 'right' else self._fid_left
        arm_ids = RIGHT_JOINT_INDICES_LIST if side == 'right' else LEFT_JOINT_INDICES_LIST
        goal = self._goal_right if side == 'right' else self._goal_left
        if goal is None:
            return None

        # Initialize joint vector
        q = q_init.copy() if q_init is not None else pin.neutral(self.model)
        for jid_idx, ros_name in enumerate(self._ros_joint_names):
            if ros_name in self._name_to_q_index:
                q[self._name_to_q_index[ros_name]] = float(current_all[jid_idx])

        # Iterative IK loop
        for _ in range(self.max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            M_cur = self.data.oMf[fid]
            R_des = self._limit_ori_step(M_cur.rotation, goal.rotation)
            T_des = SE3(R_des, goal.translation)

            # Frame Jacobian
            J6 = pin.computeFrameJacobian(self.model, self.data, q, fid, pin.LOCAL_WORLD_ALIGNED)
            J_eff = J6[:, [self._name_to_v_index[self._ros_joint_names[i]] for i in arm_ids]]

            # Pose error
            err6 = pin.log(M_cur.inverse() * T_des).vector
            err6[:3] *= self.pos_gain
            err6[3:] *= self.ori_gain

            if np.linalg.norm(err6) < self.tol:
                break

            # Adaptive damping
            lam = self.lambda_base
            if self.adaptive_damping:
                svals = np.linalg.svd(J_eff, compute_uv=False)
                sigma_min = np.min(svals) if len(svals) > 0 else 0.0
                if sigma_min < self.sigma_min_thresh:
                    frac = clamp((self.sigma_min_thresh - sigma_min) / self.sigma_min_thresh, 0, 1)
                    lam = self.lambda_base + frac * (self.lambda_max - self.lambda_base)

            # Damped least squares
            JJt = J_eff @ J_eff.T
            dq_red = J_eff.T @ np.linalg.solve(JJt + lam * np.eye(J_eff.shape[0]), err6)

            # Collision avoidance
            if getattr(self, "enable_collision_avoidance", False):
                dq_repulse = self._collision_repulsion(q)
                dq_red += dq_repulse[[self._name_to_v_index[self._ros_joint_names[i]] for i in arm_ids]]

            # Integrate step
            for i, arm_i in enumerate(arm_ids):
                step = np.clip(dq_red[i], -self.max_dq_step, self.max_dq_step)
                qi = self._name_to_q_index[self._ros_joint_names[arm_i]]
                q[qi] = np.clip(q[qi] + step, *JOINT_LIMITS_RAD[arm_i])

        # Return arm subset (7 joints)
        return np.array([float(q[self._name_to_q_index[self._ros_joint_names[i]]]) for i in arm_ids], dtype=float)
