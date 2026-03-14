#!/usr/bin/env python3
"""
Physics-based balance simulation: initial + desired orientation → LQR → IK → physics.

Convention: pitch = rotation about X (nose up/down), roll = rotation about Y (bank),
yaw about Z; front = +Y. State order [roll, pitch, yaw].

Set initial orientation (sliders), desired orientation (ref for LQR), press Start.
Physics integrates wheel torques, ball motion, platform tilt. Real-time 3D view,
state display, and Simulink-style scope plots for comparison.

Run:  python3 balance_sim_ui.py   (from simulation/ik_validation)
  or  python3 simulation/ik_validation/balance_sim_ui.py   (from project root)
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
import tkinter as tk
from tkinter import ttk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from lib.ik import ik
from lib.lqr import compute_lqr
from lib.physics_sim import PhysicsSim
from lib.viz_3d import draw_platform_and_wheels

DT = 0.0025   # physics step (400 Hz internal)
DT_MS = 20    # UI tick ~50 Hz; run multiple physics steps per tick for dense samples
PHYS_STEPS_PER_TICK = 8   # 8 * 0.0025 = 0.02 s per tick, 8 samples logged per tick
DEG_RANGE = 30
LEAN_DEG_RANGE = 15   # person lean ±15°
MAX_T = 8.0   # higher so control can overcome gravity moment
LOG_HISTORY_S = 10   # scope plot history (seconds)
SCOPE_UPDATE_EVERY = 4   # update scope plots every N ticks (smoother display)


def deg2rad(d: float) -> float:
    return math.radians(d)


class BalanceSimUI:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("Balance sim — physics + LQR → IK → motors")
        self.root.geometry("1000x560")
        self.root.resizable(True, True)

        self.physics = PhysicsSim()
        self.K = compute_lqr()
        self.running = False
        self.controller_enabled = True
        self.after_id: str | None = None
        self._last_t1 = self._last_t2 = self._last_t3 = 0.0
        self._sim_time = 0.0
        self._init_log_buffers()

        self._build_ui()
        self._build_scope_window()

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=6)
        main.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(main)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 6))
        right = ttk.Frame(main)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # ----- Initial orientation -----
        init_f = ttk.LabelFrame(left, text="Initial orientation [°] — set before Start")
        init_f.pack(fill=tk.X, pady=(0, 6))
        self.init_vars = []
        self.init_labels = []
        self.init_scales = []
        for i, name in enumerate(["Roll", "Pitch", "Yaw"]):
            row = ttk.Frame(init_f)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=f"{name}:", width=6, anchor=tk.W).pack(side=tk.LEFT)
            v = tk.DoubleVar(value=0.0)
            self.init_vars.append(v)
            lb = ttk.Label(row, text="0.0", width=6)

            def _on_slide(val: str, idx: int = i, lbl: ttk.Label = lb):
                try:
                    x = float(val)
                    lbl.config(text=f"{x:.1f}")
                    self.init_vars[idx].set(x)
                except Exception:
                    pass
                if not self.running:
                    self.lbl_orient.config(
                        text="roll={:.1f}  pitch={:.1f}  yaw={:.1f}".format(
                            self.init_vars[0].get(), self.init_vars[1].get(), self.init_vars[2].get()
                        )
                    )
                self._refresh_3d()

            s = ttk.Scale(row, from_=-DEG_RANGE, to=DEG_RANGE, variable=v, orient=tk.HORIZONTAL, length=160, command=_on_slide)
            s.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
            lb.pack(side=tk.LEFT)
            self.init_labels.append(lb)

        # ----- Desired orientation (ref for LQR) -----
        des_f = ttk.LabelFrame(left, text="Desired orientation [°] — LQR reference")
        des_f.pack(fill=tk.X, pady=(0, 6))
        self.des_vars = []
        self.des_labels = []
        for i, name in enumerate(["Roll", "Pitch", "Yaw"]):
            row = ttk.Frame(des_f)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=f"{name}:", width=6, anchor=tk.W).pack(side=tk.LEFT)
            v = tk.DoubleVar(value=0.0)
            self.des_vars.append(v)
            lb = ttk.Label(row, text="0.0", width=6)

            def _on_slide_d(val: str, idx: int = i, lbl: ttk.Label = lb):
                try:
                    x = float(val)
                    lbl.config(text=f"{x:.1f}")
                    self.des_vars[idx].set(x)
                except Exception:
                    pass

            s = ttk.Scale(row, from_=-DEG_RANGE, to=DEG_RANGE, variable=v, orient=tk.HORIZONTAL, length=160, command=_on_slide_d)
            s.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
            lb.pack(side=tk.LEFT)
            self.des_labels.append(lb)

        # ----- Lean [°] — runtime; weight disturbance, moves platform -----
        lean_f = ttk.LabelFrame(left, text="Lean [°] — person leaning (runtime)")
        lean_f.pack(fill=tk.X, pady=(0, 6))
        self.lean_vars = []
        self.lean_labels = []
        for i, name in enumerate(["Roll", "Pitch"]):
            row = ttk.Frame(lean_f)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=f"{name}:", width=6, anchor=tk.W).pack(side=tk.LEFT)
            v = tk.DoubleVar(value=0.0)
            self.lean_vars.append(v)
            lb = ttk.Label(row, text="0.0", width=6)

            def _on_lean(val: str, idx: int = i, lbl: ttk.Label = lb):
                try:
                    x = float(val)
                    lbl.config(text=f"{x:.1f}")
                    self.lean_vars[idx].set(x)
                except Exception:
                    pass
                self._refresh_3d()

            s = ttk.Scale(row, from_=-LEAN_DEG_RANGE, to=LEAN_DEG_RANGE, variable=v, orient=tk.HORIZONTAL, length=160, command=_on_lean)
            s.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
            lb.pack(side=tk.LEFT)
            self.lean_labels.append(lb)

        # ----- Buttons -----
        btn_f = ttk.Frame(left)
        btn_f.pack(fill=tk.X, pady=(4, 6))
        self.btn_start_fall = ttk.Button(btn_f, text="Start (Controller Off)", command=self._on_start_fall)
        self.btn_start_fall.pack(side=tk.LEFT, padx=2)
        self.btn_start_balance = ttk.Button(btn_f, text="Start (Controller On)", command=self._on_start_balance)
        self.btn_start_balance.pack(side=tk.LEFT, padx=2)
        self.btn_stop = ttk.Button(btn_f, text="Stop", command=self._on_stop, state=tk.DISABLED)
        self.btn_stop.pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_f, text="Reset", command=self._on_reset).pack(side=tk.LEFT, padx=2)

        # ----- State display -----
        state_f = ttk.LabelFrame(left, text="State (live)")
        state_f.pack(fill=tk.X, pady=(0, 6))
        inner = ttk.Frame(state_f)
        inner.pack(fill=tk.X)
        ttk.Label(inner, text="Orientation [°]:", anchor=tk.W).pack(anchor=tk.W)
        self.lbl_orient = ttk.Label(inner, text="roll=0.0  pitch=0.0  yaw=0.0")
        self.lbl_orient.pack(anchor=tk.W)
        ttk.Label(inner, text="Wheel θ [°]:", anchor=tk.W).pack(anchor=tk.W, pady=(4, 0))
        self.lbl_wheel = ttk.Label(inner, text="θ1=0.0  θ2=0.0  θ3=0.0")
        self.lbl_wheel.pack(anchor=tk.W)
        ttk.Label(inner, text="Ball (x,y) [m]:", anchor=tk.W).pack(anchor=tk.W, pady=(4, 0))
        self.lbl_ball = ttk.Label(inner, text="x=0.000  y=0.000")
        self.lbl_ball.pack(anchor=tk.W)
        ttk.Label(inner, text="Motors T1,T2,T3 [N·m]:", anchor=tk.W).pack(anchor=tk.W, pady=(4, 0))
        self.lbl_mot = ttk.Label(inner, text="0.00  0.00  0.00")
        self.lbl_mot.pack(anchor=tk.W)
        ttk.Label(inner, text="Correcting T1,T2,T3 [N·m]:", anchor=tk.W).pack(anchor=tk.W, pady=(4, 0))
        self.lbl_correcting = ttk.Label(inner, text="—")
        self.lbl_correcting.pack(anchor=tk.W)
        ttk.Label(inner, text="Mode:", anchor=tk.W).pack(anchor=tk.W, pady=(4, 0))
        self.lbl_mode = ttk.Label(inner, text="Idle")
        self.lbl_mode.pack(anchor=tk.W)

        # ----- 3D -----
        fig = Figure(figsize=(5.8, 4.8), dpi=100)
        self.ax = fig.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(fig, master=right)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._refresh_3d()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _init_log_buffers(self) -> None:
        max_len = int(LOG_HISTORY_S / DT) + 1
        self._t_list: list[float] = []
        self._wheel1: list[float] = []
        self._wheel2: list[float] = []
        self._wheel3: list[float] = []
        self._T1_list: list[float] = []
        self._T2_list: list[float] = []
        self._T3_list: list[float] = []
        self._roll_list: list[float] = []
        self._pitch_list: list[float] = []
        self._yaw_list: list[float] = []
        self._omega_r: list[float] = []
        self._omega_p: list[float] = []
        self._omega_y: list[float] = []
        self._Roll_T_list: list[float] = []
        self._Pitch_T_list: list[float] = []
        self._Yaw_T_list: list[float] = []
        self._log_max_len = max_len

    def _clear_log_buffers(self) -> None:
        self._t_list.clear()
        self._wheel1.clear()
        self._wheel2.clear()
        self._wheel3.clear()
        self._T1_list.clear()
        self._T2_list.clear()
        self._T3_list.clear()
        self._roll_list.clear()
        self._pitch_list.clear()
        self._yaw_list.clear()
        self._omega_r.clear()
        self._omega_p.clear()
        self._omega_y.clear()
        self._Roll_T_list.clear()
        self._Pitch_T_list.clear()
        self._Yaw_T_list.clear()

    def _build_scope_window(self) -> None:
        self.scope_win = tk.Toplevel(self.root)
        self.scope_win.title("Scope plots (Simulink-style)")
        self.scope_win.geometry("950x1100")
        self.scope_win.protocol("WM_DELETE_WINDOW", lambda: None)
        fig = Figure(figsize=(9.5, 11), dpi=100)
        # 5 scope groups, 3 subplots each (Simulink-style: one signal per subplot)
        self.scope_axs = fig.subplots(5, 3, sharex=True)
        fig.suptitle("Scope plots (Simulink-style)", fontsize=10)
        fig.tight_layout(pad=1.2, rect=[0, 0, 1, 0.98])
        self.scope_canvas = FigureCanvasTkAgg(fig, master=self.scope_win)
        self.scope_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        # Create persistent line objects for smooth updates (no clear/replot each frame)
        self._scope_lines: list[list] = []
        titles = [
            (["TorqueInput — T1", "T2", "T3"], "N·m"),
            (["Roll-Pitch-Yaw — roll", "pitch", "yaw"], "deg"),
            (["PositionOutput — θ1", "θ2", "θ3"], "deg"),
            (["AngRoll-AngPitch-AngYaw — ω_roll", "ω_pitch", "ω_yaw"], "rad/s"),
            (["RollT-PitchT-YawT — Roll_T", "Pitch_T", "Yaw_T"], "N·m"),
        ]
        for row, ((titles_row, ylabel), axs_row) in enumerate(zip(titles, self.scope_axs)):
            lines_row = []
            for col, (ax, tit) in enumerate(zip(axs_row, titles_row)):
                ax.set_xlim(0, LOG_HISTORY_S)
                ax.grid(True)
                ax.set_ylabel(ylabel if col == 0 else "", fontsize=8)
                ax.set_title(tit, fontsize=9)
                ln, = ax.plot([], [], antialiased=True)
                lines_row.append(ln)
            self._scope_lines.append(lines_row)
        self._scope_data: list[tuple[list[float], list[list[float]]]] = [
            (self._t_list, [self._T1_list, self._T2_list, self._T3_list]),
            (self._t_list, [self._roll_list, self._pitch_list, self._yaw_list]),
            (self._t_list, [self._wheel1, self._wheel2, self._wheel3]),
            (self._t_list, [self._omega_r, self._omega_p, self._omega_y]),
            (self._t_list, [self._Roll_T_list, self._Pitch_T_list, self._Yaw_T_list]),
        ]

    def _update_scope_plots(self) -> None:
        if not self._t_list:
            return
        t = np.array(self._t_list)
        t_now = t[-1]
        if t_now <= LOG_HISTORY_S:
            xmin, xmax = 0.0, max(t_now, 0.1)  # expand to fit width while time is short
        else:
            xmin, xmax = t_now - LOG_HISTORY_S, t_now  # fixed window, scroll (compressed)
        for row, (_, data_lists) in enumerate(self._scope_data):
            for col, (line, lst) in enumerate(zip(self._scope_lines[row], data_lists)):
                line.set_data(t, lst)
                ax = self.scope_axs[row, col]
                ax.set_xlim(xmin, xmax)
                ax.relim()
                ax.autoscale_view(scalex=False, scaley=True)
        self.scope_canvas.draw()

    def _clear_scope_plots(self) -> None:
        for row in range(len(self._scope_lines)):
            for col, line in enumerate(self._scope_lines[row]):
                line.set_data([], [])
                ax = self.scope_axs[row, col]
                ax.set_xlim(0, LOG_HISTORY_S)
                ax.relim()
                ax.autoscale_view(scalex=False, scaley=True)
        self.scope_canvas.draw()

    def _get_init_rad(self) -> np.ndarray:
        return np.array([deg2rad(self.init_vars[i].get()) for i in range(3)])

    def _get_desired_rad(self) -> np.ndarray:
        return np.array([deg2rad(self.des_vars[i].get()) for i in range(3)])

    def _refresh_3d(self) -> None:
        if self.running:
            r, p, y = self.physics.orientation_deg()
            th1, th2, th3 = self.physics.wheel_angles_rad()
            bx, by = self.physics.ball_position()
            brx, bry, brz = self.physics.ball_rotation_rad()
            t1, t2, t3 = self._last_t1, self._last_t2, self._last_t3
        else:
            # Init sliders: [Roll, Pitch, Yaw]. Pitch about X (nose up), roll about Y (bank), front = +Y.
            r = float(self.init_vars[0].get())
            p = float(self.init_vars[1].get())
            y = float(self.init_vars[2].get())
            th1 = th2 = th3 = 0.0
            bx = by = 0.0
            brx = bry = brz = 0.0
            t1 = t2 = t3 = 0.0
        lr = float(self.lean_vars[0].get())
        lp = float(self.lean_vars[1].get())
        # Pass (roll, pitch, yaw): Roll = about Y (bank), Pitch = about X (nose up), front = +Y.
        draw_platform_and_wheels(
            self.ax, r, p, y,
            t1, t2, t3,
            max_T=MAX_T,
            wheel_theta_rad=(th1, th2, th3),
            ball_xy=(bx, by),
            ball_rot_rad=(brx, bry, brz),
            lean_roll_deg=lr,
            lean_pitch_deg=lp,
            flip_travel=True,
        )
        if not self.running:
            self._update_correcting_torques()
        self.canvas.draw_idle()

    def _update_correcting_torques(self) -> None:
        """Show motor torques that would correct current initial orientation toward desired (LQR + IK)."""
        init_rad = self._get_init_rad()
        ref_rad = self._get_desired_rad()
        x = np.array([init_rad[0], init_rad[1], init_rad[2], 0.0, 0.0, 0.0])
        x_ref = np.array([ref_rad[0], ref_rad[1], ref_rad[2], 0.0, 0.0, 0.0])
        tau = -self.K @ (x - x_ref)
        tau = np.array([float(tau[0]), float(tau[1]), float(tau[2])])
        t1, t2, t3 = ik(tau[1], tau[0], tau[2], max_T=MAX_T)
        self.lbl_correcting.config(text="{:.2f}  {:.2f}  {:.2f}".format(t1, t2, t3))

    def _tick(self) -> None:
        if not self.running:
            return
        # State order: [roll, pitch, yaw]; pitch = X (nose up), roll = Y (bank), front = +Y
        ref_rad = self._get_desired_rad()
        x_ref = np.array([ref_rad[0], ref_rad[1], ref_rad[2], 0.0, 0.0, 0.0])
        lr = deg2rad(self.lean_vars[0].get())
        lp = deg2rad(self.lean_vars[1].get())

        for _ in range(PHYS_STEPS_PER_TICK):
            actual = self.physics.orientation_rad()
            omega = self.physics.omega
            if self.controller_enabled:
                # LQR expects x = [roll, pitch, yaw, roll_dot, pitch_dot, yaw_dot].
                # Physics omega is [pitch_dot, roll_dot, yaw_dot] (X, Y, Z).
                x = np.array([actual[0], actual[1], actual[2], omega[1], omega[0], omega[2]])
                tau = -self.K @ (x - x_ref)
                tau = np.array([float(tau[0]), float(tau[1]), float(tau[2])])
                t1, t2, t3 = ik(tau[1], tau[0], tau[2], max_T=MAX_T)
            else:
                tau = np.array([0.0, 0.0, 0.0])
                t1, t2, t3 = 0.0, 0.0, 0.0
            self._last_t1, self._last_t2, self._last_t3 = t1, t2, t3

            self.physics.step(t1, t2, t3, DT, lean_roll_rad=lr, lean_pitch_rad=lp)
            self._sim_time += DT

            # Log for scope plots. State order [roll, pitch, yaw].
            w1, w2, w3 = self.physics.wheel_angles_deg()
            r, p, y = self.physics.orientation_deg()
            self._t_list.append(self._sim_time)
            self._wheel1.append(w1)
            self._wheel2.append(w2)
            self._wheel3.append(w3)
            self._T1_list.append(t1)
            self._T2_list.append(t2)
            self._T3_list.append(t3)
            self._roll_list.append(r)
            self._pitch_list.append(p)
            self._yaw_list.append(y)
            self._omega_r.append(omega[0])
            self._omega_p.append(omega[1])
            self._omega_y.append(omega[2])
            self._Roll_T_list.append(tau[0])
            self._Pitch_T_list.append(tau[1])
            self._Yaw_T_list.append(tau[2])

        self._tick_count += 1
        for lst in [
            self._t_list, self._wheel1, self._wheel2, self._wheel3,
            self._T1_list, self._T2_list, self._T3_list,
            self._roll_list, self._pitch_list, self._yaw_list,
            self._omega_r, self._omega_p, self._omega_y,
            self._Roll_T_list, self._Pitch_T_list, self._Yaw_T_list,
        ]:
            while len(lst) > self._log_max_len:
                lst.pop(0)

        if self._tick_count % SCOPE_UPDATE_EVERY == 0:
            self._update_scope_plots()

        self.lbl_orient.config(
            text="roll={:.1f}  pitch={:.1f}  yaw={:.1f}".format(*self.physics.orientation_deg())
        )
        self.lbl_wheel.config(
            text="θ1={:.1f}°  θ2={:.1f}°  θ3={:.1f}°".format(*self.physics.wheel_angles_deg_360())
        )
        bx, by = self.physics.ball_position()
        self.lbl_ball.config(text="x={:.3f}  y={:.3f}".format(bx, by))
        self.lbl_mot.config(text="{:.2f}  {:.2f}  {:.2f}".format(t1, t2, t3))

        self._refresh_3d()
        self.after_id = self.root.after(DT_MS, self._tick)

    def _start_common(self, controller_enabled: bool) -> None:
        init = self._get_init_rad()
        self.physics.set_orientation(init[0], init[1], init[2])
        self.controller_enabled = controller_enabled
        self._clear_log_buffers()
        self._sim_time = 0.0
        self._tick_count = 0
        self._last_t1 = self._last_t2 = self._last_t3 = 0.0
        self.running = True
        self.btn_start_fall.config(state=tk.DISABLED)
        self.btn_start_balance.config(state=tk.DISABLED)
        self.btn_stop.config(state=tk.NORMAL)
        for scale in self.init_scales:
            scale.config(state=tk.DISABLED)
        self.lbl_mode.config(text="Controller OFF (fall test)" if not controller_enabled else "Controller ON (balance)")
        self.lbl_correcting.config(text="—")
        self._tick()

    def _on_start_fall(self) -> None:
        self._start_common(controller_enabled=False)

    def _on_start_balance(self) -> None:
        self._start_common(controller_enabled=True)

    def _on_stop(self) -> None:
        self.running = False
        if self.after_id:
            self.root.after_cancel(self.after_id)
            self.after_id = None
        self.btn_start_fall.config(state=tk.NORMAL)
        self.btn_start_balance.config(state=tk.NORMAL)
        self.btn_stop.config(state=tk.DISABLED)
        for scale in self.init_scales:
            scale.config(state=tk.NORMAL)
        self.lbl_mode.config(text="Idle")
        self._refresh_3d()

    def _on_reset(self) -> None:
        was_running = self.running
        self._on_stop()
        self.physics.reset()
        self._clear_log_buffers()
        self._sim_time = 0.0
        self._tick_count = 0
        for v in self.init_vars:
            v.set(0.0)
        for v in self.des_vars:
            v.set(0.0)
        for v in self.lean_vars:
            v.set(0.0)
        for lb in self.init_labels:
            lb.config(text="0.0")
        for lb in self.des_labels:
            lb.config(text="0.0")
        for lb in self.lean_labels:
            lb.config(text="0.0")
        self.lbl_orient.config(text="roll=0.0  pitch=0.0  yaw=0.0")
        self.lbl_wheel.config(text="θ1=0°  θ2=0°  θ3=0°")
        self.lbl_ball.config(text="x=0.000  y=0.000")
        self.lbl_mot.config(text="0.00  0.00  0.00")
        self.lbl_mode.config(text="Idle")
        self._last_t1 = self._last_t2 = self._last_t3 = 0.0
        self._clear_scope_plots()
        self._refresh_3d()
        if was_running:
            self._start_common(controller_enabled=self.controller_enabled)

    def _on_close(self) -> None:
        self._on_stop()
        try:
            self.scope_win.destroy()
        except Exception:
            pass
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> int:
    app = BalanceSimUI()
    app.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
