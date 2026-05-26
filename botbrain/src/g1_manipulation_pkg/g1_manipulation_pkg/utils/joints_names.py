JOINT_NAMES_ROS = {
    0: "left_hip_pitch_joint",   1: "left_hip_roll_joint",    2: "left_hip_yaw_joint",
    3: "left_knee_joint",        4: "left_ankle_pitch_joint", 5: "left_ankle_roll_joint",
    6: "right_hip_pitch_joint",  7: "right_hip_roll_joint",   8: "right_hip_yaw_joint",
    9: "right_knee_joint",      10: "right_ankle_pitch_joint",11:"right_ankle_roll_joint",
    12:"waist_yaw_joint",       13: "waist_roll_joint",      14:"waist_pitch_joint",
    15:"left_shoulder_pitch_joint", 16:"left_shoulder_roll_joint", 17:"left_shoulder_yaw_joint",
    18:"left_elbow_joint", 19:"left_wrist_roll_joint", 20:"left_wrist_pitch_joint", 21:"left_wrist_yaw_joint",
    22:"right_shoulder_pitch_joint",23:"right_shoulder_roll_joint",24:"right_shoulder_yaw_joint",
    25:"right_elbow_joint",26:"right_wrist_roll_joint",27:"right_wrist_pitch_joint",28:"right_wrist_yaw_joint",
}

JOINT_LIMITS_RAD = {
    0: (-2.5307,  2.8798),  1: (-0.5236,  2.9671),  2: (-2.7576,  2.7576),
    3: (-0.087267,2.8798),  4: (-0.87267, 0.5236),  5: (-0.2618,  0.2618),
    6: (-2.5307,  2.8798),  7: (-2.9671,  0.5236),  8: (-2.7576,  2.7576),
    9: (-0.087267,2.8798), 10:(-0.87267, 0.5236), 11:(-0.2618,  0.2618),
    12:(-2.618,   2.618),  13:(-0.52,    0.52),   14:(-0.52,    0.52),
    15:(-3.0892,  2.6704), 16:(-1.5882,  2.2515), 17:(-2.618,   2.618),
    18:(-1.0472,  2.0944), 19:(-1.972222054, 1.972222054),
    20:(-1.614429558, 1.614429558), 21:(-1.614429558, 1.614429558),
    22:(-3.0892,  2.6704), 23:(-2.2515,  1.5882), 24:(-2.618,   2.618),
    25:(-1.0472,  2.0944), 26:(-1.972222054, 1.972222054),
    27:(-1.614429558, 1.614429558), 28:(-1.614429558, 1.614429558),
}

RIGHT_JOINT_INDICES_LIST = [22, 23, 24, 25, 26, 27, 28]
LEFT_JOINT_INDICES_LIST  = [15, 16, 17, 18, 19, 20, 21]

JOINT_GROUPS = {
    "left":  LEFT_JOINT_INDICES_LIST,
    "right": RIGHT_JOINT_INDICES_LIST,
    "both":  LEFT_JOINT_INDICES_LIST + RIGHT_JOINT_INDICES_LIST,
}

JOINT_NAMES_LEFT = [
    "L Shoulder Pitch", "L Shoulder Roll", "L Shoulder Yaw",
    "L Elbow", "L Wrist Roll", "L Wrist Pitch", "L Wrist Yaw"
]
JOINT_NAMES_RIGHT = [
    "R Shoulder Pitch", "R Shoulder Roll", "R Shoulder Yaw",
    "R Elbow", "R Wrist Roll", "R Wrist Pitch", "R Wrist Yaw"
]
