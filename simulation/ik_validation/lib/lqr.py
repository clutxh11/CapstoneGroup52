"""
LQR gain computation for 3-axis ball-bot balance.
Matches compute_lqr_ballbot.m in simulation/Matlab.
Convention: pitch about X (nose up/down), roll about Y (bank), yaw about Z; front = +Y.

State: x = [roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot]
Input: u = [tau_roll, tau_pitch, tau_yaw]
Control law: u = -K @ x
Note: When building x from physics, use (roll_dot, pitch_dot, yaw_dot) = (omega[1], omega[0], omega[2])
      since physics.omega is [pitch_dot, roll_dot, yaw_dot] (X, Y, Z).
"""

from __future__ import annotations

import numpy as np
from scipy.linalg import solve_continuous_are


# Physical parameters (match user-provided setup)
# Top assembly mass = platform + person
M_ASSY = 80.0
G = 9.81
H_CM = 0.2
J_ROLL = 0.05
J_PITCH = 0.05
J_YAW = 0.03
B_ROLL = 0.5
B_PITCH = 0.5
B_YAW = 0.3

# LQR weights
Q_DIAG = np.array([500.0, 200.0, 50.0, 10.0, 10.0, 5.0])
R_SCALE = 0.05


def compute_lqr(
    m_assy: float = M_ASSY,
    g: float = G,
    h_cm: float = H_CM,
    j_roll: float = J_ROLL,
    j_pitch: float = J_PITCH,
    j_yaw: float = J_YAW,
    b_roll: float = B_ROLL,
    b_pitch: float = B_PITCH,
    b_yaw: float = B_YAW,
    q_diag: np.ndarray | None = None,
    r_scale: float = R_SCALE,
) -> np.ndarray:
    """
    Compute LQR gain K (3x6) for ball-bot.
    u = -K @ x
    """
    q_diag = q_diag if q_diag is not None else Q_DIAG.copy()
    q = np.diag(q_diag)
    r = r_scale * np.eye(3)

    g_roll = m_assy * g * h_cm / j_roll
    g_pitch = m_assy * g * h_cm / j_pitch

    a = np.array([
        [0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1],
        [g_roll, 0, 0, -b_roll / j_roll, 0, 0],
        [0, g_pitch, 0, 0, -b_pitch / j_pitch, 0],
        [0, 0, 0, 0, 0, -b_yaw / j_yaw],
    ], dtype=float)

    b = np.array([
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [1 / j_roll, 0, 0],
        [0, 1 / j_pitch, 0],
        [0, 0, 1 / j_yaw],
    ], dtype=float)

    p = solve_continuous_are(a, b, q, r)
    k = np.linalg.solve(r, b.T @ p)
    return k
