"""
Simple 3D rotational plant for testing PID + IK.
State: [roll, pitch, yaw] (rad), [omega_r, omega_p, omega_y] (rad/s).
Input: body torques [Roll_T, Pitch_T, Yaw_T] in N·m.
Decoupled axes, Euler integration.
"""

import numpy as np

# Inertia (kg·m²) and damping (N·m·s) placeholders – tune for believable response
J = np.array([0.05, 0.05, 0.03])
B = np.array([0.5, 0.5, 0.3])


class Plant:
    def __init__(self) -> None:
        self.theta = np.zeros(3)   # roll, pitch, yaw [rad]
        self.omega = np.zeros(3)   # [rad/s]

    def step(self, tau: np.ndarray, dt: float) -> None:
        """tau: [Roll_T, Pitch_T, Yaw_T]."""
        alpha = (tau - B * self.omega) / J
        self.omega += alpha * dt
        self.theta += self.omega * dt

    def state(self) -> np.ndarray:
        return self.theta.copy()

    def reset(self) -> None:
        self.theta = np.zeros(3)
        self.omega = np.zeros(3)
