"""
Discrete PID controller for roll, pitch, yaw.
Outputs body torques [Roll_T, Pitch_T, Yaw_T] in N·m.
Angles in radians internally.
"""

import numpy as np


class PID:
    def __init__(self, kp: float, ki: float, kd: float, dt: float, out_limit: float = 2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.out_limit = out_limit
        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, error: float) -> float:
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt if self.dt > 0 else 0.0
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        out = np.clip(out, -self.out_limit, self.out_limit)
        # Anti-windup: clamp integral when output saturates
        if abs(out) >= self.out_limit and (out * error > 0):
            self.integral -= error * self.dt
        return float(out)

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0


class PID3:
    """Three PIDs for roll, pitch, yaw. Outputs [Roll_T, Pitch_T, Yaw_T] in N·m."""

    def __init__(
        self,
        kp: float = 50.0,
        ki: float = 10.0,
        kd: float = 5.0,
        dt: float = 0.02,
        out_limit: float = 2.0,
    ):
        self.pids = [PID(kp, ki, kd, dt, out_limit) for _ in range(3)]
        self.dt = dt

    def step(self, ref: np.ndarray, actual: np.ndarray) -> np.ndarray:
        """ref, actual: [roll, pitch, yaw] in radians. Returns [Roll_T, Pitch_T, Yaw_T]."""
        err = ref - actual
        return np.array([p.step(e) for p, e in zip(self.pids, err)])

    def reset(self) -> None:
        for p in self.pids:
            p.reset()

    def set_dt(self, dt: float) -> None:
        self.dt = dt
        for p in self.pids:
            p.dt = dt
