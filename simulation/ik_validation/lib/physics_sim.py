"""
Simplified physics engine for ball-bot balance simulation.

State: platform orientation [roll, pitch, yaw] + angular velocity;
       ball position (x, y), velocity (vx, vy);
       wheel angles θ1, θ2, θ3 and angular velocities ω1, ω2, ω3.

Wheel torques T1, T2, T3 → wheel dynamics → torque on ball (via FK) →
ball acceleration → platform tilt (inverted-pendulum style). Simplified
for demo; not a replacement for full multibody.
"""

from __future__ import annotations

import math
import numpy as np

from .ik import WHEEL_DEG, fk

# Platform inertia (kg·m²) and damping (N·m·s)
J_PLAT = np.array([0.05, 0.05, 0.03])
B_PLAT = np.array([0.5, 0.5, 0.3])
# Mass model (user-provided values)
BALL_M = 15.0
PLATFORM_M = 30.0
PERSON_M = 50.0
WHEEL_M_TOTAL = 15.0
M_WHEEL = WHEEL_M_TOTAL / 3.0
M_ASSY = PLATFORM_M + PERSON_M  # top assembly mass used in gravity term

# Gravity moment: inverted pendulum.
# tau_grav = +GRAV_SCALE * M*g*H*sin(θ), where M is the top assembly mass.
G = 9.81
H_CM = 0.2      # m, CoM above ball center (user-provided)
GRAV_SCALE = 0.35   # 0–1; lower = easier to balance (control can overcome)

# Wheel: 15 kg total -> 5 kg each, 14 cm diam -> R 0.07 m.
WHEEL_R = 0.07
J_WHEEL = 0.03
B_WHEEL = 2.0

# Ball: 15 kg, 0.6 m diameter -> R 0.3 m
BALL_R = 0.3
K_TAU_TO_A = 0.15
BALL_DAMP = 0.3

# Wheel-ball friction (user-provided)
MU_STATIC = 0.7
MU_DYNAMIC = 0.4
SLIP_OMEGA_THRESH = 0.2  # rad/s, small threshold to switch static -> dynamic model

# Person lean: extra torque on platform (N·m per rad lean). Positive lean = tip that way.
# Higher = person weight shifts CoM more.
K_LEAN = 6.0

# Ball–wheel kinematic coupling: ball velocity drags wheels (friction contact).
# Desired wheel omega from ball (vx, vy) so wheels roll with the ball; coupling torque drags wheel toward it.
K_BALL_DRAG_WHEEL = 8.0   # N·m·s per rad/s error (stiffness of rolling constraint)

# --- For Simulink MATLAB function (updated user masses) ---
# Ball 15 kg, platform 30 kg, person 50 kg, wheels 15 kg total, H_CM=0.2 m.
IK_SCALE_RECOMMENDED = 2.0
IK_MAX_T_RECOMMENDED = 15.0


def _euler_zyx(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Rotation matrix: pitch about X (nose up/down), roll about Y (bank), yaw about Z. Forward = +Y.
    R = Rz(yaw) @ Ry(roll) @ Rx(pitch). Body: X = right, Y = forward, Z = up."""
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]], dtype=float)
    Ry = np.array([[cr, 0, sr], [0, 1, 0], [-sr, 0, cr]], dtype=float)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return Rz @ Ry @ Rx


class PhysicsSim:
    def __init__(self) -> None:
        # Platform: orientation (rad), angular velocity (rad/s)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.omega = np.zeros(3)  # [ωr, ωp, ωy]

        # Ball: position (m), velocity (m/s). z = BALL_R fixed.
        self.ball_x = 0.0
        self.ball_y = 0.0
        self.ball_vx = 0.0
        self.ball_vy = 0.0
        # Ball visual rotation state (rad), derived from planar rolling velocity
        self.ball_rot = np.zeros(3)  # [rx, ry, rz]

        # Wheels: angle (rad), angular velocity (rad/s)
        self.wheel_theta = np.zeros(3)
        self.wheel_omega = np.zeros(3)

    def orientation_rad(self) -> np.ndarray:
        """Current platform orientation [roll, pitch, yaw] in rad. Pitch about X (nose up/down), roll about Y (bank), forward = +Y."""
        return np.array([self.roll, self.pitch, self.yaw], dtype=float)

    def orientation_deg(self) -> tuple[float, float, float]:
        """Current platform orientation (roll, pitch, yaw) in degrees. Forward = +Y."""
        return (
            math.degrees(self.roll),
            math.degrees(self.pitch),
            math.degrees(self.yaw),
        )

    def wheel_angles_rad(self) -> tuple[float, float, float]:
        """Wheel angles θ1, θ2, θ3 in rad."""
        return (float(self.wheel_theta[0]), float(self.wheel_theta[1]), float(self.wheel_theta[2]))

    def wheel_angles_deg(self) -> tuple[float, float, float]:
        """Wheel angles in degrees."""
        t = self.wheel_angles_rad()
        return (math.degrees(t[0]), math.degrees(t[1]), math.degrees(t[2]))

    def wheel_angles_deg_360(self) -> tuple[float, float, float]:
        """Wheel angles in degrees, wrapped to [0, 360)."""
        d = self.wheel_angles_deg()
        return (d[0] % 360.0, d[1] % 360.0, d[2] % 360.0)

    def ball_position(self) -> tuple[float, float]:
        """Ball center (x, y) in m."""
        return (self.ball_x, self.ball_y)

    def ball_rotation_rad(self) -> tuple[float, float, float]:
        """Ball visual rotation [rx, ry, rz] in rad."""
        return (float(self.ball_rot[0]), float(self.ball_rot[1]), float(self.ball_rot[2]))

    def set_orientation(self, roll_rad: float, pitch_rad: float, yaw_rad: float) -> None:
        """Set platform orientation (rad). Zeros angular velocities."""
        self.roll = float(roll_rad)
        self.pitch = float(pitch_rad)
        self.yaw = float(yaw_rad)
        self.omega.fill(0.0)
        self.ball_vx = 0.0
        self.ball_vy = 0.0
        self.ball_rot.fill(0.0)
        self.wheel_omega.fill(0.0)

    def reset(self) -> None:
        """Reset to upright, zero motion."""
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.omega.fill(0.0)
        self.ball_x = 0.0
        self.ball_y = 0.0
        self.ball_vx = 0.0
        self.ball_vy = 0.0
        self.ball_rot.fill(0.0)
        self.wheel_theta.fill(0.0)
        self.wheel_omega.fill(0.0)

    def step(
        self,
        T1: float,
        T2: float,
        T3: float,
        dt: float,
        lean_roll_rad: float = 0.0,
        lean_pitch_rad: float = 0.0,
    ) -> None:
        """
        Integrate one timestep. Motor torques T1, T2, T3 (N·m).
        lean_roll_rad, lean_pitch_rad: person lean (rad); adds disturbance torque.
        """
        # Wheel-ball traction limit from Coulomb friction.
        # Approximate normal load per wheel as evenly distributed total weight.
        normal_per_wheel = (BALL_M + M_ASSY + WHEEL_M_TOTAL) * G / 3.0
        tau_fric_static = MU_STATIC * normal_per_wheel * WHEEL_R
        tau_fric_dynamic = MU_DYNAMIC * normal_per_wheel * WHEEL_R

        # Desired wheel angular velocity from ball motion.
        # Tangent at wheel azimuth θ (from +Y): t_hat = (-cos θ, sin θ).
        # This is generated from ik.WHEEL_DEG so wheel geometry stays consistent.
        vx, vy = self.ball_vx, self.ball_vy
        if abs(WHEEL_R) < 1e-9:
            omega_des = np.zeros(3)
        else:
            omega_des = np.array(
                [
                    ((-math.cos(math.radians(deg))) * vx + (math.sin(math.radians(deg))) * vy) / WHEEL_R
                    for deg in WHEEL_DEG
                ],
                dtype=float,
            )

        # Wheel dynamics with simple static/dynamic slip model.
        # T_contact is transmitted through wheel-ball contact (limited by friction).
        T_contact = np.zeros(3, dtype=float)
        for i in range(3):
            T = [T1, T2, T3][i]
            om = self.wheel_omega[i]

            # Static-like region: commanded torque under static limit and wheel speed small.
            if abs(T) <= tau_fric_static and abs(om) < SLIP_OMEGA_THRESH:
                t_eff = T
            else:
                # Slip region: torque transfer limited by dynamic friction.
                t_eff = math.copysign(min(abs(T), tau_fric_dynamic), T)

            T_contact[i] = t_eff
            # Motor torque + damping + ball-drag coupling (wheels are dragged by ball motion via friction).
            tau_drag = K_BALL_DRAG_WHEEL * (omega_des[i] - om)
            alpha = (T + tau_drag - B_WHEEL * om) / J_WHEEL
            self.wheel_omega[i] += alpha * dt
            self.wheel_theta[i] += self.wheel_omega[i] * dt

        # Body torques: FK returns [Roll_T, Pitch_T, Yaw_T]. Body axes: X = pitch, Y = roll, Z = yaw.
        tau = fk(float(T_contact[0]), float(T_contact[1]), float(T_contact[2]))
        tau_body = np.array([tau[1], tau[0], tau[2]], dtype=float)  # [Pitch_T, Roll_T, Yaw_T] for X,Y,Z

        # Gravity: pitch about X (nose up/down), roll about Y (bank)
        tau_grav = GRAV_SCALE * np.array([
            M_ASSY * G * H_CM * math.sin(self.pitch),
            M_ASSY * G * H_CM * math.sin(self.roll),
            0.0,
        ], dtype=float)

        # Person lean: lean_roll → torque about Y, lean_pitch → torque about X. Negate pitch for nose-up.
        tau_lean = K_LEAN * np.array([-lean_pitch_rad, lean_roll_rad, 0.0], dtype=float)

        # Platform angular acceleration: omega[0]=pitch_dot (X), omega[1]=roll_dot (Y), omega[2]=yaw_dot (Z)
        alpha_plat = (tau_body + tau_grav + tau_lean - B_PLAT * self.omega) / J_PLAT
        self.omega += alpha_plat * dt
        self.pitch += self.omega[0] * dt
        self.roll += self.omega[1] * dt
        self.yaw += self.omega[2] * dt

        # Ball acceleration from horizontal component of body torque in world frame.
        # Sign so motion matches lean: lean forward → move forward (+Y); lean right → move right (+X).
        # Pitch: positive tau_pitch (nose-up) → ball forward → ay = +K*tau_world[0].
        # Roll: negative tau_roll (correct right tilt) → ball right → ax = -K*tau_world[1].
        R = _euler_zyx(self.roll, self.pitch, self.yaw)
        tau_world = R @ tau_body
        ax = -K_TAU_TO_A * tau_world[1] / max(BALL_M, 0.1)   # roll → ball X (sign for stability + lean-right→move-right)
        ay = K_TAU_TO_A * tau_world[0] / max(BALL_M, 0.1)   # pitch → ball Y (lean-forward→move-forward)
        self.ball_vx += ax * dt
        self.ball_vy += ay * dt
        self.ball_vx *= max(0.0, 1.0 - BALL_DAMP * dt)
        self.ball_vy *= max(0.0, 1.0 - BALL_DAMP * dt)
        self.ball_x += self.ball_vx * dt
        self.ball_y += self.ball_vy * dt

        # Visual ball rotation cue from rolling velocity on ground plane.
        # For rolling on plane: v = omega × (-R zhat) => omega_x = vy/R, omega_y = -vx/R
        if BALL_R > 1e-9:
            self.ball_rot[0] += (self.ball_vy / BALL_R) * dt
            self.ball_rot[1] += (-self.ball_vx / BALL_R) * dt
