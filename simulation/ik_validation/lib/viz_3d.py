"""
3D visualization: platform orientation + wheel spin arrows (T1, T2, T3).

Convention: pitch = rotation about X (nose up/down), roll = rotation about Y (bank), yaw about Z.
Frame: +X right, +Y forward (front), +Z up. Omniwheels tilted α = 25.659°.
Arrow = spin axis (tilted); +T = arrow direction, -T = opposite. Length ∝ |T|.
"""

from __future__ import annotations

import math
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from .ik import ALPHA_DEG, WHEEL_DEG, fk

# Legacy: check_travel_dir imports this for frame label.
FLIP_X = False

# Wheel azimuth from +Y (degrees): 120° spacing with Wheel 2 on -Y axis.
# Source of truth is ik.WHEEL_DEG to keep visualization and IK aligned.
WHEEL_RADIUS = 0.4
DISC_RADIUS = 0.14   # visual size of tilted wheel disc
DISC_DEPTH = 0.04   # thickness (extent along spin axis) for 3D look
ARROW_SCALE = 0.2   # max arrow length when |T| = MAX_T
ARC_DEG = 240       # curved spin arc (degrees)
SPIN_ARROW_LEN = 0.28   # length of spin-direction arrowhead on wheel
PLATFORM_RXY = (0.55, 0.7)  # half-extents (X = lateral, Y = forward)

# Ball (below platform) and ground — 0.6 m diam → R 0.3 m
BALL_R = 0.3
BALL_CENTER = np.array([0.0, 0.0, -BALL_R])
GROUND_Z = -0.65
GROUND_HALF = 1.4
TAU_ARROW_SCALE = 0.2    # resultant torque arrow length scale
TRAVEL_ARROW_LEN = 0.35  # travel-direction arrow on ground
TILT_ARROW_LEN = 0.45    # green arrow = direction of tilt (resultant of roll + pitch)

# Person on platform (lean visualization)
PERSON_H = 0.5       # body height (m)
HEAD_R = 0.08        # head radius
LEAN_ARROW_LEN = 0.35   # lean-direction arrow length

# Frame: +X right, +Y forward (front), +Z up. No flip or swap.

# Visual-only gains so small physical motion is visible in real time.
WHEEL_ROT_VIS_GAIN = 24.0
BALL_ROT_VIS_GAIN = 80.0
BALL_POS_VIS_GAIN = 120.0   # scale ball (and platform) XY position for visible translation

_view_initialized = False


def euler_zyx_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Rotation matrix: pitch about X (nose up/down), roll about Y (bank), yaw about Z. Forward = +Y.
    R = Rz(yaw) @ Ry(roll) @ Rx(pitch). Call with (roll_rad, pitch_rad, yaw_rad)."""
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]], dtype=float)
    Ry = np.array([[cr, 0, sr], [0, 1, 0], [-sr, 0, cr]], dtype=float)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return Rz @ Ry @ Rx


def compute_travel_direction(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    tau_body: np.ndarray,
) -> np.ndarray | None:
    """Unit 2D travel = horizontal τ (same line as resultant torque on ball). No slip ⇒ system moves along τ."""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)   # positive pitch_deg = nose up (about X)
    yaw = math.radians(yaw_deg)
    R = euler_zyx_matrix(roll, pitch, yaw)
    tau_world = (R @ tau_body).astype(float)
    tx, ty = tau_world[0], tau_world[1]
    d_xy = np.array([tx, ty], dtype=float)  # travel ∥ τ (same line)
    n = np.linalg.norm(d_xy)
    if n < 1e-12:
        return None
    return d_xy / n


def compute_tilt_direction(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray | None:
    """Unit 2D tilt direction (green arrow, downward slope). Same as viz green arrow."""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)   # positive pitch_deg = nose up
    yaw = math.radians(yaw_deg)
    R = euler_zyx_matrix(roll, pitch, yaw)
    up_world = R @ np.array([0.0, 0.0, 1.0])
    tilt_xy = np.array([up_world[0], up_world[1]], dtype=float)
    n = np.linalg.norm(tilt_xy)
    if n < 1e-12:
        return None
    return tilt_xy / n


def compute_resultant_torque_direction(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    tau_body: np.ndarray,
) -> np.ndarray | None:
    """Unit 3D resultant torque (τ) direction in world frame (orange arrow on ball).
    Returns (dx, dy, dz) or None if zero."""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)   # positive pitch_deg = nose up
    yaw = math.radians(yaw_deg)
    R = euler_zyx_matrix(roll, pitch, yaw)
    tau_world = (R @ tau_body).astype(float)
    n = np.linalg.norm(tau_world)
    if n < 1e-12:
        return None
    return tau_world / n


def platform_mesh_local() -> np.ndarray:
    """Platform corners in local XY (z=0). Simple rectangle. +Y = front."""
    rx, ry = PLATFORM_RXY
    return np.array([
        [-rx, -ry, 0],
        [rx, -ry, 0],
        [rx, ry, 0],
        [-rx, ry, 0],
        [-rx, -ry, 0],
    ])


def wheel_poses_local() -> list[tuple[np.ndarray, np.ndarray]]:
    """(position, spin_axis) in local frame. Spin axis accounts for wheel tilt α.

    Omniwheels are tilted α = 25.659° from horizontal. Roll/pitch use horizontal
    component (cos α × tangent); yaw uses vertical component (sin α × Z). Spin axis
    = cos(α) × tangent + sin(α) × (0,0,1), normalized.
    """
    alpha_rad = math.radians(ALPHA_DEG)
    ca = math.cos(alpha_rad)
    sa = math.sin(alpha_rad)
    out = []
    for deg in WHEEL_DEG:
        th = math.radians(deg)
        # Position: azimuth from +Y, in platform plane
        px = math.sin(th) * WHEEL_RADIUS
        py = math.cos(th) * WHEEL_RADIUS
        pos = np.array([px, py, 0.0])
        # Tangent in XY (horizontal part of spin axis)
        tx = -math.cos(th)
        ty = math.sin(th)
        horizontal = np.array([tx, ty, 0.0])
        # Tilted spin axis: cos(α) × horizontal + sin(α) × Z (yaw component)
        axis = ca * horizontal + sa * np.array([0.0, 0.0, 1.0])
        n = np.linalg.norm(axis)
        if n > 1e-9:
            axis = axis / n
        else:
            axis = horizontal / np.linalg.norm(horizontal)
        out.append((pos, axis))
    return out


def transform_points(R: np.ndarray, pts: np.ndarray) -> np.ndarray:
    return (R @ pts.T).T


def _draw_ball(ax, ball_xy: tuple[float, float] = (0.0, 0.0)) -> None:
    """Draw sphere (ball) below platform. Center at (ball_xy[0], ball_xy[1], -BALL_R)."""
    bx, by = ball_xy[0], ball_xy[1]
    r = BALL_R
    u = np.linspace(0, 2 * np.pi, 24)
    v = np.linspace(0, np.pi, 14)
    x = r * np.outer(np.cos(u), np.sin(v)) + bx
    y = r * np.outer(np.sin(u), np.sin(v)) + by
    z = r * np.outer(np.ones_like(u), np.cos(v)) - BALL_R
    ax.plot_surface(x, y, z, color="silver", alpha=0.5, edgecolor="gray", linewidth=0.3)


def _draw_ground(ax) -> None:
    """Draw ground plane below ball."""
    h = GROUND_HALF
    z = GROUND_Z
    pts = np.array([[-h, -h, z], [h, -h, z], [h, h, z], [-h, h, z]])
    poly = Poly3DCollection([pts], alpha=0.2, facecolor="green", edgecolor="darkgreen", linewidths=0.5)
    ax.add_collection3d(poly)


def _draw_person_and_lean(
    ax,
    R: np.ndarray,
    offset: np.ndarray,
    lean_roll_deg: float,
    lean_pitch_deg: float,
) -> None:
    """Draw simple person on platform + lean-direction arrow.
    Body: X = right, Y = forward. lean_pitch → forward/back (Y), lean_roll → left/right (X)."""
    lr = math.radians(lean_roll_deg)
    lp = math.radians(lean_pitch_deg)
    H = PERSON_H
    # Body Y = forward, X = lateral
    dy = H * math.tan(lp) if abs(math.cos(lp)) > 1e-6 else 0.0  # lean_pitch → forward/back (Y)
    dx = H * math.tan(lr) if abs(math.cos(lr)) > 1e-6 else 0.0  # lean_roll → left/right (X)
    base = np.array([0.0, 0.0, 0.0])
    top = np.array([dx, dy, H])
    pts = np.array([base, top])
    pts_w = transform_points(R, pts) + offset
    ax.plot(pts_w[:, 0], pts_w[:, 1], pts_w[:, 2], color="navy", linewidth=4, solid_capstyle="round", label="person")
    # Head: small sphere at top
    cx, cy, cz = pts_w[1, 0], pts_w[1, 1], pts_w[1, 2]
    u = np.linspace(0, 2 * np.pi, 16)
    v = np.linspace(0, np.pi, 10)
    hx = HEAD_R * np.outer(np.cos(u), np.sin(v)) + cx
    hy = HEAD_R * np.outer(np.sin(u), np.sin(v)) + cy
    hz = HEAD_R * np.outer(np.ones_like(u), np.cos(v)) + cz
    ax.plot_surface(hx, hy, hz, color="wheat", alpha=0.9, edgecolor="tan", linewidth=0.2)
    # Lean arrow: direction in body frame (forward = Y, lateral = X)
    sy, sx = math.sin(lp), math.sin(lr)
    n = math.hypot(sx, sy)
    if n < 1e-9:
        return
    d_local = np.array([sx / n, sy / n, 0.0])
    d_world = R @ d_local
    L = LEAN_ARROW_LEN
    ax.quiver(
        cx, cy, cz,
        L * d_world[0], L * d_world[1], L * d_world[2],
        color="darkblue", arrow_length_ratio=0.25, linewidth=2.5, label="lean",
    )


def _wheel_plane_uv(axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Two orthonormal vectors in the plane perpendicular to axis (wheel plane)."""
    a = np.asarray(axis, dtype=float).ravel()
    a = a / (np.linalg.norm(a) + 1e-12)
    if abs(a[2]) < 0.9:
        u = np.cross(a, [0, 0, 1])
    else:
        u = np.cross(a, [1, 0, 0])
    u = u / (np.linalg.norm(u) + 1e-12)
    v = np.cross(a, u)
    v = v / (np.linalg.norm(v) + 1e-12)
    return u, v


def draw_wheel_rotation_marker(
    ax,
    center: np.ndarray,
    axis: np.ndarray,
    theta_rad: float,
    color: str,
) -> None:
    """Draw high-contrast marker on wheel face to show rotation clearly."""
    u, v = _wheel_plane_uv(axis)
    r = DISC_RADIUS
    h = DISC_DEPTH
    face_center = center + (h / 2) * axis
    rim_pt = face_center + r * (math.cos(theta_rad) * u + math.sin(theta_rad) * v)
    rim_pt2 = face_center + 0.65 * r * (math.cos(theta_rad + math.pi / 2) * u + math.sin(theta_rad + math.pi / 2) * v)
    # white underlay + colored overlay for visibility on any wheel color
    ax.plot([face_center[0], rim_pt[0]], [face_center[1], rim_pt[1]], [face_center[2], rim_pt[2]],
            color="white", linewidth=4, solid_capstyle="round")
    ax.plot([face_center[0], rim_pt[0]], [face_center[1], rim_pt[1]], [face_center[2], rim_pt[2]],
            color=color, linewidth=2.2, solid_capstyle="round")
    ax.plot([face_center[0], rim_pt2[0]], [face_center[1], rim_pt2[1]], [face_center[2], rim_pt2[2]],
            color="white", linewidth=3, solid_capstyle="round", alpha=0.9)
    ax.plot([face_center[0], rim_pt2[0]], [face_center[1], rim_pt2[1]], [face_center[2], rim_pt2[2]],
            color=color, linewidth=1.8, solid_capstyle="round", alpha=0.95)
    ax.scatter([rim_pt[0]], [rim_pt[1]], [rim_pt[2]], color="yellow", s=24, depthshade=False)


def _draw_ball_axes(
    ax,
    ball_xy: tuple[float, float],
    ball_rot_rad: tuple[float, float, float] | None,
) -> None:
    """Draw a small rotating XYZ triad at ball center as ball-rotation cue."""
    if ball_rot_rad is None:
        return
    bx, by = ball_xy
    center = np.array([bx, by, -BALL_R], dtype=float)
    rx, ry, rz = ball_rot_rad
    rx *= BALL_ROT_VIS_GAIN
    ry *= BALL_ROT_VIS_GAIN
    rz *= BALL_ROT_VIS_GAIN
    Rb = euler_zyx_matrix(rx, ry, rz)
    L = 0.14
    basis = np.eye(3)
    colors = ["red", "green", "blue"]
    labels = ["X", "Y", "Z"]
    for i in range(3):
        d = Rb @ basis[:, i]
        p0 = center.copy()
        p1 = center + L * d
        dv = p1 - p0
        ax.quiver(p0[0], p0[1], p0[2], dv[0], dv[1], dv[2],
                  color=colors[i], linewidth=2.2, arrow_length_ratio=0.22)
        ax.text(p1[0], p1[1], p1[2], labels[i], color=colors[i], fontsize=8, fontweight="bold")


def _draw_wheel_disc_and_spin(
    ax,
    center: np.ndarray,
    axis: np.ndarray,
    Ti: float,
    color: str,
    max_T: float,
) -> None:
    """Draw tilted thick disc (wheel) + curved spin arc. +T = one way, -T = opposite."""
    u, v = _wheel_plane_uv(axis)
    r = DISC_RADIUS
    h = DISC_DEPTH
    n = 32
    th = np.linspace(0, 2 * np.pi, n, endpoint=True)
    circ = np.outer(np.cos(th), u) + np.outer(np.sin(th), v)

    # Two parallel circles offset along spin axis (front and back faces)
    top = center + (h / 2) * axis + r * circ
    bot = center - (h / 2) * axis + r * circ
    ax.plot(top[:, 0], top[:, 1], top[:, 2], color=color, linewidth=1.5, alpha=0.9)
    ax.plot(bot[:, 0], bot[:, 1], bot[:, 2], color=color, linewidth=1.5, alpha=0.9)

    # Rim quads (cylindrical surface) for 3D depth
    rim_polys = []
    for i in range(n):
        j = (i + 1) % n
        q = np.array([top[i], top[j], bot[j], bot[i]])
        rim_polys.append(q)
    rim_col = Poly3DCollection(rim_polys, alpha=0.35, facecolor=color, edgecolor=color, linewidths=0.5)
    ax.add_collection3d(rim_col)
    # Front/back face fills
    ax.add_collection3d(Poly3DCollection([top], alpha=0.3, facecolor=color, edgecolor="none"))
    ax.add_collection3d(Poly3DCollection([bot], alpha=0.3, facecolor=color, edgecolor="none"))

    # Curved spin arc + arrowhead on front (top) face
    s = np.sign(Ti) if Ti != 0 else 0
    mag = min(abs(Ti) / max(max_T, 1e-6), 1.0)
    if s == 0 or mag < 0.05:
        return
    deg = np.linspace(0, s * np.radians(ARC_DEG), max(2, int(24 * mag)), endpoint=True)
    arc = center + (h / 2) * axis + r * (np.outer(np.cos(deg), u) + np.outer(np.sin(deg), v))
    ax.plot(arc[:, 0], arc[:, 1], arc[:, 2], color=color, linewidth=2.5, solid_capstyle="round")

    # Arrowhead at arc end
    t_end = deg[-1]
    tangent = (-np.sin(t_end) * u + np.cos(t_end) * v) * s * SPIN_ARROW_LEN
    tip = arc[-1]
    ax.quiver(
        tip[0], tip[1], tip[2],
        tangent[0], tangent[1], tangent[2],
        color=color, arrow_length_ratio=0.35, linewidth=2,
    )


def draw_platform_and_wheels(
    ax,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    t1: float,
    t2: float,
    t3: float,
    max_T: float = 2.0,
    wheel_theta_rad: tuple[float, float, float] | None = None,
    ball_xy: tuple[float, float] | None = None,
    ball_rot_rad: tuple[float, float, float] | None = None,
    lean_roll_deg: float | None = None,
    lean_pitch_deg: float | None = None,
    flip_travel: bool = False,
) -> None:
    """Clear ax and draw platform, wheels, ball, ground, torque and travel arrows.
    If wheel_theta_rad is given, draw rotation markers (θ1, θ2, θ3) on each wheel.
    If ball_xy is given, draw ball and platform at (bx, by); otherwise (0, 0).
    If lean_roll_deg / lean_pitch_deg are given, draw person and lean-direction arrow.
    If flip_travel, travel arrow matches Segway-like motion (same direction as lean)."""
    ax.cla()
    bx_raw = 0.0 if ball_xy is None else ball_xy[0]
    by_raw = 0.0 if ball_xy is None else ball_xy[1]
    bx = bx_raw * BALL_POS_VIS_GAIN
    by = by_raw * BALL_POS_VIS_GAIN
    offset = np.array([bx, by, 0.0])

    # Pitch about X, roll about Y, yaw about Z. Forward = +Y.
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)   # positive = nose up
    yaw = -math.radians(yaw_deg)     # inverted so yaw slider rotates opposite direction
    R = euler_zyx_matrix(roll, pitch, yaw)

    # Resultant torque on ball: FK(T1,T2,T3) -> [Roll_T, Pitch_T, Yaw_T]
    tau = fk(t1, t2, t3)
    tau_body = np.array([tau[0], tau[1], tau[2]])
    tau_world = R @ tau_body

    # Ground plane (draw first, behind)
    _draw_ground(ax)

    # Ball below platform at (bx, by)
    _draw_ball(ax, (bx, by))
    _draw_ball_axes(ax, (bx, by), ball_rot_rad)

    # Resultant torque arrow on ball (from ball center)
    bc = np.array([bx, by, -BALL_R], dtype=float)
    t_norm = np.linalg.norm(tau_world)
    if t_norm > 1e-9:
        d = (TAU_ARROW_SCALE * min(t_norm / 2.0, 1.0)) * (tau_world / t_norm)
        ax.quiver(bc[0], bc[1], bc[2], d[0], d[1], d[2],
                  color="orangered", arrow_length_ratio=0.2, linewidth=2.5, label="τ ball")

    # Force (Travel) = horizontal τ (flip_travel: same direction as lean, e.g. balance sim)
    force = compute_travel_direction(roll_deg, pitch_deg, -yaw_deg, tau_body)
    if force is not None:
        if flip_travel:
            force = -force
        d_xy = TRAVEL_ARROW_LEN * force
        ax.quiver(bx, by, GROUND_Z + 0.02, d_xy[0], d_xy[1], 0,
                  color="darkviolet", arrow_length_ratio=0.25, linewidth=2.5, label="Force (Travel)")

    # Green arrow = tilt (downward slope), origin at platform center
    up_world = R @ np.array([0.0, 0.0, 1.0])
    tilt_xy = np.array([up_world[0], up_world[1]], dtype=float)
    n_tilt = np.linalg.norm(tilt_xy)
    if n_tilt > 1e-9:
        tilt_unit = tilt_xy / n_tilt
        tilt_scaled = TILT_ARROW_LEN * tilt_unit
        ax.quiver(bx, by, 0, tilt_scaled[0], tilt_scaled[1], 0,
                  color="darkgreen", arrow_length_ratio=0.2, linewidth=3, label="tilt")

    # Platform, offset by ball_xy. +Y = forward.
    pts = platform_mesh_local()
    pts_r = transform_points(R, pts) + offset
    ax.plot(pts_r[:, 0], pts_r[:, 1], pts_r[:, 2], "b-", linewidth=2, label="platform")
    v = pts_r[:4]
    poly = Poly3DCollection([v], alpha=0.25, facecolor="cyan", edgecolor="blue")
    ax.add_collection3d(poly)

    # Person + lean arrow (if lean given)
    if lean_roll_deg is not None and lean_pitch_deg is not None:
        _draw_person_and_lean(ax, R, offset, lean_roll_deg, lean_pitch_deg)

    # Labels: front = +Y, back = -Y
    nose_local = np.array([[0, PLATFORM_RXY[1] + 0.12, 0]])
    nose = (transform_points(R, nose_local) + offset)[0]
    ax.text(nose[0], nose[1], nose[2], " FRONT", fontsize=11, color="darkgreen", fontweight="bold")
    tail_local = np.array([[0, -PLATFORM_RXY[1] - 0.1, 0]])
    tail = (transform_points(R, tail_local) + offset)[0]
    ax.text(tail[0], tail[1], tail[2], " BACK ", fontsize=10, color="gray", ha="center")

    # Wheels: tilted disc (orientation) + curved spin arc (rotation direction), offset by ball_xy
    wheels = wheel_poses_local()
    torques = [t1, t2, t3]
    colors = ["#c22", "#2a2", "#22c"]
    for i, ((pos, axis), Ti) in enumerate(zip(wheels, torques)):
        pos_r = (R @ pos + offset)
        axis_r = (R @ axis)
        _draw_wheel_disc_and_spin(ax, pos_r, axis_r, Ti, colors[i], max_T)
        if wheel_theta_rad is not None and i < len(wheel_theta_rad):
            th = wheel_theta_rad[i] * WHEEL_ROT_VIS_GAIN
            draw_wheel_rotation_marker(ax, pos_r, axis_r, th, colors[i])
        # Label above wheel
        ax.text(pos_r[0], pos_r[1], pos_r[2] + 0.1, f"T{i+1}", fontsize=9, color=colors[i], ha="center")

    ax.set_xlabel("X (right)")
    ax.set_ylabel("Y (front)")
    ax.set_zlabel("Z (up)")
    half = 1.2
    ax.set_xlim3d(bx - half, bx + half)
    ax.set_ylim3d(by - half, by + half)
    ax.set_zlim3d(-1.2, 1.1)
    ax.set_box_aspect([1, 1, 1])
    title = "Ball + platform + wheels │ green = tilt │ Orange = τ │ Purple = travel"
    if lean_roll_deg is not None and lean_pitch_deg is not None:
        title += " │ blue = lean"
    ax.set_title(title)
    global _view_initialized
    if not _view_initialized:
        ax.view_init(elev=22, azim=55)
        _view_initialized = True
