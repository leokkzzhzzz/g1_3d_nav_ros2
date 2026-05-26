import math
import numpy as np
import pinocchio as pin

from g1_manipulation_pkg.utils.joints_names import JOINT_LIMITS_RAD


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def wrap_to_pi(a):
    a = (a + np.pi) % (2 * np.pi) - np.pi
    return a


def mat_to_quat_wxyz(R):
    q = pin.Quaternion(R)
    return np.array([q.w, q.x, q.y, q.z], dtype=float)


def quat_wxyz_to_matrix(qwxyz):
    w, x, y, z = qwxyz
    q = pin.Quaternion(w, x, y, z)
    return q.matrix()


def quat_hemisphere(q0, q1):
    if np.dot(q0, q1) < 0.0:
        return -q1
    return q1


def quat_slerp(q0, q1, t):
    q0 = np.array(q0, dtype=float) / np.linalg.norm(q0)
    q1 = np.array(q1, dtype=float) / np.linalg.norm(q1)
    q1 = quat_hemisphere(q0, q1)
    dotv = np.clip(np.dot(q0, q1), -1.0, 1.0)
    if dotv > 0.9995:
        out = q0 + t * (q1 - q0)
        return out / np.linalg.norm(out)
    theta_0 = math.acos(dotv)
    sin_0 = math.sin(theta_0)
    theta = theta_0 * t
    s0 = math.sin(theta_0 - theta) / sin_0
    s1 = math.sin(theta) / sin_0
    return (s0 * q0 + s1 * q1)


def yaw_from_R(R):
    return math.atan2(R[1, 0], R[0, 0])


def rotz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]], dtype=float)


def clamp_joint_vector(q_vals, joint_id_list):
    out = []
    for ii, jidx in enumerate(joint_id_list):
        lo, hi = JOINT_LIMITS_RAD[jidx]
        out.append(float(np.clip(q_vals[ii], lo, hi)))
    return np.array(out, dtype=float)
