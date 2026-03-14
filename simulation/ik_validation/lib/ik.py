"""
Inverse kinematics for 3-wheel ball balancer (matches Simulink MATLAB Function block).

Wheel layout: 120° spacing. Layout rotated so Wheel 2 is on -Y axis.
Wheel 1 @ 60°, Wheel 2 @ 180° (-Y), Wheel 3 @ 300°. Front = -X. alpha = 25.659°.
"""

import math
import numpy as np

ALPHA_DEG = 25.659
CA = math.cos(math.radians(ALPHA_DEG))
SA = math.sin(math.radians(ALPHA_DEG))

# Wheel azimuths from +Y in platform XY (deg), 120° spacing with T2 on -Y.
WHEEL_DEG = [60.0, 180.0, 300.0]

# Row i = [cos(θ_i)*CA, sin(θ_i)*CA, SA] for wheels at 60°, 180°, 300° (T2 in -Y)
C1, S1 = math.cos(math.radians(WHEEL_DEG[0])), math.sin(math.radians(WHEEL_DEG[0]))
C2, S2 = math.cos(math.radians(WHEEL_DEG[1])), math.sin(math.radians(WHEEL_DEG[1]))
C3, S3 = math.cos(math.radians(WHEEL_DEG[2])), math.sin(math.radians(WHEEL_DEG[2]))

# IK matrix: [T1; T2; T3] = A @ [Roll_T; Pitch_T; Yaw_T]
A = np.array([
    [C1 * CA, S1 * CA, SA],
    [C2 * CA, S2 * CA, SA],
    [C3 * CA, S3 * CA, SA],
], dtype=float)


def ik(pitch_T: float, roll_T: float, yaw_T: float, max_T: float = 2.0):
    """Compute motor torques T1, T2, T3 from desired body torques (Pitch, Roll, Yaw)."""
    t1 = (C1 * CA * roll_T) + (S1 * CA * pitch_T) + (SA * yaw_T)
    t2 = (C2 * CA * roll_T) + (S2 * CA * pitch_T) + (SA * yaw_T)
    t3 = (C3 * CA * roll_T) + (S3 * CA * pitch_T) + (SA * yaw_T)
    t1 = max(min(t1, max_T), -max_T)
    t2 = max(min(t2, max_T), -max_T)
    t3 = max(min(t3, max_T), -max_T)
    return (t1, t2, t3)


def fk(T1: float, T2: float, T3: float) -> np.ndarray:
    """Forward kinematics: wheel torques -> [Roll_T, Pitch_T, Yaw_T]."""
    A_inv = np.linalg.inv(A)
    return A_inv @ np.array([T1, T2, T3])
