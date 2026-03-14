#!/usr/bin/env python3
"""
Run the Arduino control pipeline on simulated orientation data.

Uses the same logic and constants as:
  controlCode/arduinoCode/control_helper.cpp
  controlCode/arduinoCode/motor_control_helper.cpp
  controlCode/arduinoCode/arduinoCode.ino

Outputs: tau_roll, tau_pitch, tau_yaw | T1, T2, T3 | v1, v2, v3 | sent_v1, sent_v2, sent_v3

Usage:
  python3 run_arduino_pipeline.py [--roll DEG] [--pitch DEG] [--yaw DEG] [--omega_r R] [--omega_p P] [--omega_y Y]
  python3 run_arduino_pipeline.py --csv orientation_log.csv [--out results.csv]
  python3 run_arduino_pipeline.py --sim 5 [--out comparison.csv]   # run balance sim 5s, compare Arduino vs sim motors
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

# Add simulation/ik_validation so we can import ik (and optionally run sim)
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent.parent
IK_PATH = PROJECT_ROOT / "simulation" / "ik_validation"
if str(IK_PATH) not in sys.path:
    sys.path.insert(0, str(IK_PATH))

import numpy as np
from lib.ik import ik

# ----- Match control_helper.cpp -----
K_LQR = np.array([
    [343.068682, 0.0, 0.0, 14.815250, 0.0, 0.0],
    [0.0, 326.183053, 0.0, 0.0, 14.760023, 0.0],
    [0.0, 0.0, 31.622777, 0.0, 0.0, 9.798879],
], dtype=float)

LQR_REMAP_SWAP = 0
LQR_DEADZONE_DEG = 0.0
LQR_DEADZONE_RAD = math.radians(LQR_DEADZONE_DEG)
TAU_SCALE = 0.01
K_TAU_TO_VEL = 2.0
VEL_MAX_LQR = 10.0
VEL_SCALE = 0.25
IK_MAX_TORQUE = 2.0

# ----- Match arduinoCode.ino -----
IGNORE_YAW_IN_CHASSIS = True
INVERT_ROLL = True

# Hardware wiring / IMU mixup: swap motor 1↔2 and roll↔pitch channels when True.
SWAP_M1_M2_AND_ROLL_PITCH = True

# ----- Match motor_control_helper.cpp -----
MOTOR_INVERT_V1_V2_SIGN = 1  # 1 = send (-v1, -v2, v3) to ODrive


def deg2rad(d: float) -> float:
    return math.radians(d)


def arduino_pipeline(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    omega_roll: float = 0.0,
    omega_pitch: float = 0.0,
    omega_yaw: float = 0.0,
) -> dict:
    """
    Run the same pipeline as Arduino: state -> LQR -> TAU_SCALE -> IK -> v, clamp, VEL_SCALE.
    Returns dict with tau_roll, tau_pitch, tau_yaw, T1, T2, T3, v1, v2, v3, sent_v1, sent_v2, sent_v3.
    """
    roll_rad = deg2rad(roll_deg)
    pitch_rad = deg2rad(pitch_deg)
    yaw_rad = deg2rad(yaw_deg)

    x = np.array([
        roll_rad, pitch_rad, yaw_rad,
        omega_roll, omega_pitch, omega_yaw,
    ], dtype=float)

    if INVERT_ROLL:
        x[0] = -x[0]
        x[3] = -x[3]

    # Optional: swap logical roll/pitch axes to match mixed IMU mounting
    if SWAP_M1_M2_AND_ROLL_PITCH:
        x[0], x[1] = x[1], x[0]
        x[3], x[4] = x[4], x[3]

    x_ref = np.zeros(6)
    if IGNORE_YAW_IN_CHASSIS:
        x_ref[2] = x[2]
        x_ref[5] = x[5]

    e = x - x_ref
    if abs(e[0]) < LQR_DEADZONE_RAD:
        e[0] = 0.0
    if abs(e[1]) < LQR_DEADZONE_RAD:
        e[1] = 0.0
    if abs(e[2]) < LQR_DEADZONE_RAD:
        e[2] = 0.0

    tau = K_LQR @ e   # +K: positive error -> positive body torque
    tau_roll = float(tau[0])
    tau_pitch = float(tau[1])
    tau_yaw = float(tau[2])

    if LQR_REMAP_SWAP:
        tau_roll, tau_pitch = tau_pitch, tau_roll

    tau_r_ik = tau_roll * TAU_SCALE
    tau_p_ik = tau_pitch * TAU_SCALE
    tau_y_ik = tau_yaw * TAU_SCALE

    # Arduino IK order is (roll_T, pitch_T, yaw_T). Python ik(pitch_T, roll_T, yaw_T).
    t1, t2, t3 = ik(tau_p_ik, tau_r_ik, tau_y_ik, max_T=IK_MAX_TORQUE)

    # Motor wiring swap: logical motor-1 command goes to physical motor 2 and vice versa.
    if SWAP_M1_M2_AND_ROLL_PITCH:
        t1, t2 = t2, t1

    v1 = K_TAU_TO_VEL * t1
    v2 = K_TAU_TO_VEL * t2
    v3 = K_TAU_TO_VEL * t3
    v1 = max(-VEL_MAX_LQR, min(VEL_MAX_LQR, v1))
    v2 = max(-VEL_MAX_LQR, min(VEL_MAX_LQR, v2))
    v3 = max(-VEL_MAX_LQR, min(VEL_MAX_LQR, v3))
    v1 *= VEL_SCALE
    v2 *= VEL_SCALE
    v3 *= VEL_SCALE

    if MOTOR_INVERT_V1_V2_SIGN:
        sent_v1, sent_v2, sent_v3 = -v1, -v2, v3
    else:
        sent_v1, sent_v2, sent_v3 = v1, v2, v3

    return {
        "roll_deg": roll_deg,
        "pitch_deg": pitch_deg,
        "yaw_deg": yaw_deg,
        "omega_roll": omega_roll,
        "omega_pitch": omega_pitch,
        "omega_yaw": omega_yaw,
        "tau_roll": tau_roll,
        "tau_pitch": tau_pitch,
        "tau_yaw": tau_yaw,
        "T1": t1,
        "T2": t2,
        "T3": t3,
        "v1": v1,
        "v2": v2,
        "v3": v3,
        "sent_v1": sent_v1,
        "sent_v2": sent_v2,
        "sent_v3": sent_v3,
    }


def sim_pipeline_one(
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
    omega_pitch: float,
    omega_roll: float,
    omega_yaw: float,
    K: np.ndarray,
) -> tuple[float, float, float]:
    """
    One step of the simulation controller: x = [roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot],
    x_ref = 0, tau = -K @ x, then ik(tau_pitch, tau_roll, tau_yaw) -> T1, T2, T3.
    Returns (T1, T2, T3) as used in physics (no TAU_SCALE; sim uses full LQR output).
    """
    x = np.array([roll_rad, pitch_rad, yaw_rad, omega_roll, omega_pitch, omega_yaw], dtype=float)
    x_ref = np.zeros(6)
    tau = -K @ (x - x_ref)
    tau = np.array([float(tau[0]), float(tau[1]), float(tau[2])])
    t1, t2, t3 = ik(tau[1], tau[0], tau[2], max_T=8.0)
    return (t1, t2, t3)


def run_sim_and_compare(duration_s: float, dt: float, out_path: Path | None) -> None:
    """Run balance sim for duration_s, log state and motors; run Arduino pipeline on same state; compare."""
    from lib.physics_sim import PhysicsSim
    from lib.lqr import compute_lqr

    K_sim = compute_lqr()
    physics = PhysicsSim()
    physics.set_orientation(0.0, math.radians(3.0), 0.0)  # small initial pitch

    rows = []
    t = 0.0
    while t < duration_s:
        r, p, y = physics.orientation_rad()
        omega = physics.omega
        # Sim controller: state order for LQR is [roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot]
        T1_sim, T2_sim, T3_sim = sim_pipeline_one(
            r, p, y, omega[0], omega[1], omega[2], K_sim
        )
        physics.step(T1_sim, T2_sim, T3_sim, dt)

        # Arduino pipeline on same state (degrees, and omega in rad/s: roll_dot, pitch_dot, yaw_dot)
        ard = arduino_pipeline(
            math.degrees(r), math.degrees(p), math.degrees(y),
            omega[1], omega[0], omega[2],
        )

        rows.append({
            "t": t,
            "roll_deg": math.degrees(r),
            "pitch_deg": math.degrees(p),
            "yaw_deg": math.degrees(y),
            "T1_sim": T1_sim,
            "T2_sim": T2_sim,
            "T3_sim": T3_sim,
            "T1_arduino": ard["T1"],
            "T2_arduino": ard["T2"],
            "T3_arduino": ard["T3"],
            "v1": ard["v1"],
            "v2": ard["v2"],
            "v3": ard["v3"],
        })
        t += dt

    # Print summary
    print("Ran sim for {:.2f} s, {} steps.".format(duration_s, len(rows)))
    if rows:
        r0 = rows[0]
        print("First row: roll={:.2f} pitch={:.2f} yaw={:.2f} -> T_sim=({:.3f},{:.3f},{:.3f}) T_arduino=({:.3f},{:.3f},{:.3f}) v=({:.3f},{:.3f},{:.3f})".format(
            r0["roll_deg"], r0["pitch_deg"], r0["yaw_deg"],
            r0["T1_sim"], r0["T2_sim"], r0["T3_sim"],
            r0["T1_arduino"], r0["T2_arduino"], r0["T3_arduino"],
            r0["v1"], r0["v2"], r0["v3"],
        ))

    if out_path:
        with open(out_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print("Wrote {}".format(out_path))


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Run Arduino motor pipeline on simulated orientation; optionally compare with sim."
    )
    ap.add_argument("--roll", type=float, default=0.5, help="Roll (deg)")
    ap.add_argument("--pitch", type=float, default=10.0, help="Pitch (deg)")
    ap.add_argument("--yaw", type=float, default=0.5, help="Yaw (deg)")
    ap.add_argument("--omega_r", type=float, default=0.0, help="omega_roll (rad/s)")
    ap.add_argument("--omega_p", type=float, default=0.0, help="omega_pitch (rad/s)")
    ap.add_argument("--omega_y", type=float, default=0.0, help="omega_yaw (rad/s)")
    ap.add_argument("--tilt", type=float, default=None, metavar="DEG",
                    help="Continuous tilt mode: pitch=DEG, roll=0, yaw=0, zero rate. Shows steady-state torques.")
    ap.add_argument("--steady-state", action="store_true",
                    help="Continuous tilt: use --roll/--pitch/--yaw with zero angular velocity.")
    ap.add_argument("--csv", type=Path, default=None, help="Input CSV: roll_deg, pitch_deg, yaw_deg [, omega_r, omega_p, omega_y]")
    ap.add_argument("--sim", type=float, default=None, help="Run balance sim for N seconds and compare Arduino vs sim motors")
    ap.add_argument("--out", type=Path, default=None, help="Output CSV for --csv or --sim")
    args = ap.parse_args()

    if args.sim is not None:
        run_sim_and_compare(args.sim, 0.0025, args.out)
        return 0

    if args.csv is not None:
        if not args.csv.exists():
            print("Error: CSV not found:", args.csv, file=sys.stderr)
            return 1
        rows = []
        with open(args.csv) as f:
            r = csv.DictReader(f)
            for row in r:
                roll = float(row.get("roll_deg", row.get("roll", 0)))
                pitch = float(row.get("pitch_deg", row.get("pitch", 0)))
                yaw = float(row.get("yaw_deg", row.get("yaw", 0)))
                o_r = float(row.get("omega_r", row.get("omega_roll", 0)))
                o_p = float(row.get("omega_p", row.get("omega_pitch", 0)))
                o_y = float(row.get("omega_y", row.get("omega_yaw", 0)))
                out = arduino_pipeline(roll, pitch, yaw, o_r, o_p, o_y)
                rows.append(out)
        print("Processed {} rows from {}".format(len(rows), args.csv))
        if rows:
            print("roll_deg pitch_deg yaw_deg | tau_roll tau_pitch tau_yaw | T1 T2 T3 | v1 v2 v3 | sent_v1 sent_v2 sent_v3")
            for o in rows[:10]:
                print("{:.2f} {:.2f} {:.2f} | {:.3f} {:.3f} {:.3f} | {:.3f} {:.3f} {:.3f} | {:.3f} {:.3f} {:.3f} | {:.3f} {:.3f} {:.3f}".format(
                    o["roll_deg"], o["pitch_deg"], o["yaw_deg"],
                    o["tau_roll"], o["tau_pitch"], o["tau_yaw"],
                    o["T1"], o["T2"], o["T3"],
                    o["v1"], o["v2"], o["v3"],
                    o["sent_v1"], o["sent_v2"], o["sent_v3"],
                ))
            if len(rows) > 10:
                print("... and {} more".format(len(rows) - 10))
        if args.out:
            with open(args.out, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
            print("Wrote {}".format(args.out))
        return 0

    # Continuous tilt / steady-state mode
    if args.tilt is not None:
        args.steady_state = True
        args.roll = 0.0
        args.pitch = float(args.tilt)
        args.yaw = 0.0
        args.omega_r = args.omega_p = args.omega_y = 0.0

    if args.steady_state:
        args.omega_r = args.omega_p = args.omega_y = 0.0

    # Single point (or steady-state)
    out = arduino_pipeline(
        args.roll, args.pitch, args.yaw,
        args.omega_r, args.omega_p, args.omega_y,
    )

    if args.steady_state:
        print("Steady-state / continuous tilt — platform held at constant orientation, zero angular velocity.")
        print("  Orientation: roll={:.2f}° pitch={:.2f}° yaw={:.2f}°".format(
            out["roll_deg"], out["pitch_deg"], out["yaw_deg"],
        ))
    else:
        print("Arduino pipeline (orientation deg, omega rad/s):")
        print("  roll={:.2f} pitch={:.2f} yaw={:.2f}  omega_r={:.4f} omega_p={:.4f} omega_y={:.4f}".format(
            out["roll_deg"], out["pitch_deg"], out["yaw_deg"],
            out["omega_roll"], out["omega_pitch"], out["omega_yaw"],
        ))

    print("  Body torques (before TAU_SCALE): tau_roll={:.3f} tau_pitch={:.3f} tau_yaw={:.3f}  (N·m)".format(
        out["tau_roll"], out["tau_pitch"], out["tau_yaw"],
    ))
    print("  Wheel torques (after TAU_SCALE + IK): T1={:.3f} T2={:.3f} T3={:.3f}  (N·m)".format(
        out["T1"], out["T2"], out["T3"],
    ))
    print("  Velocity command: v1={:.3f} v2={:.3f} v3={:.3f}".format(out["v1"], out["v2"], out["v3"]))
    print("  Sent to ODrive: sent_v1={:.3f} sent_v2={:.3f} sent_v3={:.3f}".format(
        out["sent_v1"], out["sent_v2"], out["sent_v3"],
    ))
    return 0


if __name__ == "__main__":
    sys.exit(main())
