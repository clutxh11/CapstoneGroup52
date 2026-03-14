#!/usr/bin/env python3
"""
Simple GUI: motor positions M1, M2, M3 and axis triad (X, Y, Z) with orientation sliders.

Convention (matches your Simulink rotation):
  +X = front (nose).  Z = up.
  +Y goes through M1 and M3 (triangle top).  -Y goes straight through M2 (triangle bottom).
  Motors at 120° spacing from +X: M1 at 150°, M2 at 270° (-Y), M3 at 30°.

Torque display: uses lib.lqr (same formulation as simulation/Matlab/compute_lqr_ballbot.m:
  u = -K*x, same A/B/Q/R structure). Parameters (J, H_CM, M) are from lib/lqr.py, not the
  Matlab files. For Simulink’s time-varying K from lean, see simulation/Matlab/lqr_gain_from_lean.m
  and compute_lqr_lookup.m.

Convention vs Simulink:
  - Torques match lqr_gain_from_lean.m: u = K*(x - x_ref). With x_ref=0, we show tau = K@x so
    positive pitch (nose up) -> positive Pitch_T, same sign as the RollT-PitchT-YawT scope (+50 for 5°).
  - Magnitude may differ: viz uses lib/lqr (small J); Simulink uses compute_lqr_lookup (larger J).
  - TorqueInput scope = wheel torques T1,T2,T3 (e.g. T1 +10, T2 ~0, T3 -10 for pitch 5°).

Run:  python3 platform_axes_viz.py   (from simulation/ik_validation or project root)
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

from lib.lqr import compute_lqr
from lib.ik import ik

# Optional: Arduino pipeline comparison (controlCode/arduinoCode via testScript replica)
_HAS_ARDUINO_PIPELINE = False
try:
    _PROJECT_ROOT = Path(__file__).resolve().parents[2]
    _ARDUINO_TESTSCRIPT = _PROJECT_ROOT / "controlCode" / "testScript"
    if str(_ARDUINO_TESTSCRIPT) not in sys.path:
        sys.path.insert(0, str(_ARDUINO_TESTSCRIPT))
    from run_arduino_pipeline import arduino_pipeline as arduino_pipeline  # type: ignore

    _HAS_ARDUINO_PIPELINE = True
except Exception:
    _HAS_ARDUINO_PIPELINE = False

# Steady-state LQR: x_ref = 0, tau = -K @ x. Scale before IK (match Arduino).
TAU_SCALE = 0.01
IK_MAX_T = 2.0
K_LQR = compute_lqr()

# Motor positions: angle in degrees from +X (front). +Y through M1 & M3, -Y through M2.
M1_DEG = 150.0  # left side, +Y side of triangle
M2_DEG = 270.0  # on -Y (bottom of triangle)
M3_DEG = 30.0   # right side, +Y side of triangle
R = 0.4   # platform radius (arbitrary for viz)
AXIS_LEN = 0.6
SLIDER_DEG_RANGE = 45.0   # ±45° for roll, pitch, yaw


def euler_xyz_matrix(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    """Rotation matrix: XYZ order (roll about X, pitch about Y, yaw about Z).
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll). Body axes: X=front, Y=left, Z=up."""
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return Rz @ Ry @ Rx


def motor_positions_local() -> np.ndarray:
    """Motor positions in local (body) frame, z=0. Shape (3, 3)."""
    out = []
    for deg in [M1_DEG, M2_DEG, M3_DEG]:
        rad = math.radians(deg)
        x = R * math.cos(rad)
        y = R * math.sin(rad)
        out.append([x, y, 0.0])
    return np.array(out)


def compute_torques(roll_deg: float, pitch_deg: float, yaw_deg: float) -> tuple[float, float, float, float, float, float]:
    """Body torques matching Simulink lqr_gain_from_lean.m: u = K*(x - x_ref), x_ref=0 => tau = K@x.
    So positive pitch (nose up) gives positive Pitch_T like the Simulink scope (e.g. +50 for 5°)."""
    roll_rad = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)
    yaw_rad = math.radians(yaw_deg)
    x = np.array([roll_rad, pitch_rad, yaw_rad, 0.0, 0.0, 0.0], dtype=float)
    tau = K_LQR @ x  # match Simulink: u = K*(x - x_ref), same sign as RollT-PitchT-YawT scope
    tau_roll, tau_pitch, tau_yaw = float(tau[0]), float(tau[1]), float(tau[2])
    t1, t2, t3 = ik(
        tau_pitch * TAU_SCALE,
        tau_roll * TAU_SCALE,
        tau_yaw * TAU_SCALE,
        max_T=IK_MAX_T,
    )
    return (tau_roll, tau_pitch, tau_yaw, t1, t2, t3)


def draw_3d_rotated(ax_3d: Axes3D, roll_deg: float, pitch_deg: float, yaw_deg: float) -> None:
    """Draw 3D axes, rotated platform triangle, and motor points."""
    ax_3d.cla()
    roll_rad = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)
    yaw_rad = math.radians(yaw_deg)
    R_mat = euler_xyz_matrix(roll_rad, pitch_rad, yaw_rad)

    # Fixed world axis triad
    ax_3d.quiver(0, 0, 0, AXIS_LEN, 0, 0, color="red", arrow_length_ratio=0.15, linewidth=2)
    ax_3d.quiver(0, 0, 0, 0, AXIS_LEN, 0, color="green", arrow_length_ratio=0.15, linewidth=2)
    ax_3d.quiver(0, 0, 0, 0, 0, AXIS_LEN, color="blue", arrow_length_ratio=0.15, linewidth=2)
    ax_3d.text(AXIS_LEN * 1.15, 0, 0, "X (front)", fontsize=9)
    ax_3d.text(0, AXIS_LEN * 1.15, 0, "Y (through M1&M3)", fontsize=9)
    ax_3d.text(0, 0, AXIS_LEN * 1.15, "Z (up)", fontsize=9)

    # Motor positions in local frame, then rotate
    pts_local = motor_positions_local()   # (3, 3)
    pts_rotated = (R_mat @ pts_local.T).T   # (3, 3)

    # Translucent triangle between M1, M2, M3
    tri_verts = pts_rotated
    poly = Poly3DCollection(
        [tri_verts],
        alpha=0.35,
        facecolor="cyan",
        edgecolor="blue",
        linewidths=2,
    )
    ax_3d.add_collection3d(poly)

    # Motor markers and labels
    for i, (name, pt) in enumerate(zip(["M1", "M2", "M3"], pts_rotated)):
        ax_3d.scatter(pt[0], pt[1], pt[2], s=120, edgecolors="black", linewidths=2, zorder=10)
        ax_3d.text(pt[0], pt[1], pt[2], f"  {name}", fontsize=11, fontweight="bold")

    ax_3d.set_xlim(-0.8, 0.8)
    ax_3d.set_ylim(-0.8, 0.8)
    ax_3d.set_zlim(-0.2, 0.8)
    ax_3d.set_xlabel("X")
    ax_3d.set_ylabel("Y")
    ax_3d.set_zlabel("Z")
    ax_3d.set_title(f"3D — Roll={roll_deg:.1f}° Pitch={pitch_deg:.1f}° Yaw={yaw_deg:.1f}°")
    ax_3d.set_box_aspect([1, 1, 1])


def draw_rotation_diagram(ax_diag) -> None:
    """Single wheel with smooth curved arrow showing positive spin direction (view from front)."""
    ax_diag.set_aspect("equal")
    ax_diag.set_xlim(-0.4, 0.4)
    ax_diag.set_ylim(-0.4, 0.4)
    ax_diag.axis("off")
    r_wheel = 0.2
    th = np.linspace(0, 2 * np.pi, 64)
    ax_diag.plot(r_wheel * np.cos(th), r_wheel * np.sin(th), "k-", linewidth=2)
    # Smooth curved arrow: FancyArrowPatch with arc so the curve is clean
    r_arc = r_wheel * 0.88
    start_ang = 0.35 * np.pi
    end_ang = 1.65 * np.pi
    pos_start = (r_arc * np.cos(start_ang), r_arc * np.sin(start_ang))
    pos_end = (r_arc * np.cos(end_ang), r_arc * np.sin(end_ang))
    arc = FancyArrowPatch(
        pos_start,
        pos_end,
        arrowstyle="->,head_width=0.12,head_length=0.06",
        connectionstyle="arc3,rad=0.55",
        color="blue",
        linewidth=2.5,
        mutation_scale=10,
    )
    ax_diag.add_patch(arc)
    ax_diag.text(0, -r_wheel - 0.12, "positive spin\n(view from front)", fontsize=8, ha="center", color="blue")
    ax_diag.text(0, r_wheel + 0.08, "front", fontsize=8, ha="center", color="gray")


def main() -> None:
    fig = plt.figure(figsize=(11, 6))
    ax_top = fig.add_subplot(121)
    ax_3d = fig.add_subplot(122, projection="3d")
    fig.suptitle("Platform: +Y through M1 & M3, -Y through M2 — sliders rotate 3D view")

    # ---- Top-down (XY) reference ----
    ax_top.set_aspect("equal")
    ax_top.set_xlim(-1.2, 1.2)
    ax_top.set_ylim(-1.2, 1.2)
    ax_top.axhline(0, color="gray", linewidth=0.5)
    ax_top.axvline(0, color="gray", linewidth=0.5)
    th = np.linspace(0, 2 * np.pi, 64)
    ax_top.plot(R * np.cos(th), R * np.sin(th), "k-", linewidth=2, label="platform")
    for name, deg in [("M1", M1_DEG), ("M2", M2_DEG), ("M3", M3_DEG)]:
        rad = math.radians(deg)
        x = R * math.cos(rad)
        y = R * math.sin(rad)
        ax_top.plot(x, y, "o", markersize=14, markeredgecolor="black", markeredgewidth=2)
        ax_top.annotate(name, (x, y), xytext=(6, 6), textcoords="offset points", fontsize=12, fontweight="bold")
    ax_top.arrow(0, 0, AXIS_LEN, 0, head_width=0.06, head_length=0.05, fc="red", ec="red", linewidth=2)
    ax_top.arrow(0, 0, 0, AXIS_LEN, head_width=0.06, head_length=0.05, fc="green", ec="green", linewidth=2)
    ax_top.text(AXIS_LEN + 0.1, 0, "+X (front)", fontsize=10, color="red")
    ax_top.text(0.05, AXIS_LEN + 0.05, "+Y (M1&M3)", fontsize=10, color="green")
    ax_top.text(-AXIS_LEN - 0.25, 0, "-X (back)", fontsize=10, color="darkred")
    ax_top.text(0.02, -AXIS_LEN - 0.15, "-Y (M2)", fontsize=10, color="darkgreen")
    ax_top.set_xlabel("X")
    ax_top.set_ylabel("Y")
    ax_top.set_title("Top-down (reference)")

    # Sliders and bottom panel (torques + diagram)
    plt.subplots_adjust(bottom=0.42)
    sleft, sbot, swidth, sheight = 0.2, 0.22, 0.6, 0.02
    ax_roll = fig.add_axes([sleft, sbot + 2 * sheight + 0.02, swidth, sheight])
    ax_pitch = fig.add_axes([sleft, sbot + sheight + 0.01, swidth, sheight])
    ax_yaw = fig.add_axes([sleft, sbot, swidth, sheight])
    slider_roll = Slider(ax_roll, "Roll [°]", -SLIDER_DEG_RANGE, SLIDER_DEG_RANGE, valinit=0, valstep=0.5)
    slider_pitch = Slider(ax_pitch, "Pitch [°]", -SLIDER_DEG_RANGE, SLIDER_DEG_RANGE, valinit=0, valstep=0.5)
    slider_yaw = Slider(ax_yaw, "Yaw [°]", -SLIDER_DEG_RANGE, SLIDER_DEG_RANGE, valinit=0, valstep=0.5)

    # Rotation diagram (right of sliders)
    ax_diag = fig.add_axes([0.52, 0.02, 0.2, 0.16])
    draw_rotation_diagram(ax_diag)

    txt_motor = fig.text(
        0.04, 0.17, "MATLAB  T1 = 0.00  T2 = 0.00  T3 = 0.00  (N·m)", fontsize=10, family="monospace"
    )
    txt_body = fig.text(
        0.04, 0.13, "MATLAB  Roll_T = 0.00  Pitch_T = 0.00  Yaw_T = 0.00  (N·m)", fontsize=10, family="monospace"
    )
    txt_motor_ard = fig.text(
        0.04, 0.09, "ARDUINO T1 =   --    T2 =   --    T3 =   --    (N·m)", fontsize=10, family="monospace"
    )
    txt_body_ard = fig.text(
        0.04, 0.05, "ARDUINO Roll_T =   --    Pitch_T =   --    Yaw_T =   --    (N·m)", fontsize=10, family="monospace"
    )

    def on_slider_change(_val: float) -> None:
        r = slider_roll.val
        p = slider_pitch.val
        y = slider_yaw.val
        draw_3d_rotated(ax_3d, r, p, y)
        tau_r, tau_p, tau_y, t1, t2, t3 = compute_torques(r, p, y)
        txt_motor.set_text(f"MATLAB  T1 = {t1:6.2f}  T2 = {t2:6.2f}  T3 = {t3:6.2f}  (N·m)")
        txt_body.set_text(f"MATLAB  Roll_T = {tau_r:7.2f}  Pitch_T = {tau_p:7.2f}  Yaw_T = {tau_y:7.2f}  (N·m)")

        if _HAS_ARDUINO_PIPELINE:
            ard = arduino_pipeline(r, p, y, 0.0, 0.0, 0.0)
            txt_motor_ard.set_text(
                f"ARDUINO T1 = {ard['T1']:6.2f}  T2 = {ard['T2']:6.2f}  T3 = {ard['T3']:6.2f}  (N·m)"
            )
            txt_body_ard.set_text(
                f"ARDUINO Roll_T = {ard['tau_roll']:7.2f}  Pitch_T = {ard['tau_pitch']:7.2f}  Yaw_T = {ard['tau_yaw']:7.2f}  (N·m)"
            )
        else:
            txt_motor_ard.set_text("ARDUINO T1 =   --    T2 =   --    T3 =   --    (N·m)   [import failed]")
            txt_body_ard.set_text("ARDUINO Roll_T =   --    Pitch_T =   --    Yaw_T =   --    (N·m)   [import failed]")
        fig.canvas.draw_idle()

    slider_roll.on_changed(on_slider_change)
    slider_pitch.on_changed(on_slider_change)
    slider_yaw.on_changed(on_slider_change)

    # Initial 3D draw and torque display
    draw_3d_rotated(ax_3d, 0.0, 0.0, 0.0)
    on_slider_change(0.0)

    plt.show()


if __name__ == "__main__":
    main()
