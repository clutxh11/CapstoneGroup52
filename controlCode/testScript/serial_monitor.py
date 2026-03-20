#!/usr/bin/env python3
"""
Read serial output from the ball-bot Arduino and display it.
Encoder values (e1, e2, e3) are included in the Arduino printRow output.
By default opens a GUI window; use --no-gui for terminal-only.

Usage:
  python3 serial_monitor.py                    # GUI, auto-detect port
  python3 serial_monitor.py --port /dev/ttyACM0
  python3 serial_monitor.py --list             # list serial ports
  python3 serial_monitor.py --no-gui           # stream to terminal only
  python3 serial_monitor.py --csv log.csv      # also log data rows to CSV
  python3 serial_monitor.py --baud 115200      # default is 115200
"""

import argparse
import os
import queue
import re
import sys
import threading
from collections import deque

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Install pyserial: pip install pyserial", file=sys.stderr)
    sys.exit(1)

# Arduino Serial.begin(115200)
DEFAULT_BAUD = 115200

# Data row: space-separated floats/ints; header line contains "omega_R" or "Roll"
# New format: ... | v1 v2 v3 | v1_des v2_des v3_des | e1 e2 e3 | p1 p2 p3
# Old format (no v_des): ... | v1 v2 v3 | e1 e2 e3 | p1 p2 p3
DATA_ROW_PATTERN_NEW = re.compile(
    r"^\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+\|\s*"   # roll pitch yaw
    r"([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+\|\s*"       # omega_r omega_p omega_y
    r"([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+\|\s*"       # tau_r tau_p tau_y
    r"([01])\s+([01])\s+([01])\s+\|\s*"                 # n0 n1 n2
    r"([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+\|\s*"       # v1 v2 v3
    r"([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+\|\s*"       # v1_des v2_des v3_des
    r"([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+\|\s*"       # e1 e2 e3
    r"([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*$"           # p1 p2 p3
)
DATA_ROW_PATTERN_OLD = re.compile(
    r"^\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+\|\s*"   # roll pitch yaw
    r"([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+\|\s*"       # omega_r omega_p omega_y
    r"([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+\|\s*"       # tau_r tau_p tau_y
    r"([01])\s+([01])\s+([01])\s+\|\s*"                 # n0 n1 n2
    r"([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+\|\s*"       # v1 v2 v3
    r"([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+\|\s*"       # e1 e2 e3
    r"([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*$"           # p1 p2 p3
)

# TUNE line from Arduino: "TUNE M=45.0 J=32.0 H=0.32 X=0.000 | Qa=600 Qr=200 R=0.070 | KPh=10.50 KP=1.50 KI=0.20 KD=0.08"
TUNE_KEYS = ["M", "J", "H", "X", "Qa", "Qr", "R", "KPh", "KP", "KI", "KD"]


def parse_tune_line(line: str) -> dict | None:
    """Parse a TUNE line from Arduino; returns dict of key -> float or None if not a TUNE line."""
    s = line.strip()
    if not s.startswith("TUNE "):
        return None
    out = {}
    for key, val in re.findall(r"(\w+)=([-\d.]+)", s):
        if key in TUNE_KEYS:
            try:
                out[key] = float(val)
            except ValueError:
                pass
    return out if out else None


def list_ports():
    for p in serial.tools.list_ports.comports():
        print(f"  {p.device}\t{p.description}")


def find_likely_port():
    ports = list(serial.tools.list_ports.comports())
    # Prefer USB serial (Teensy, Arduino) by name
    for p in ports:
        if any(x in (p.description or "").lower() for x in ("usb", "teensy", "arduino", "serial", "usb serial")):
            return p.device
    if ports:
        return ports[0].device
    return None


def parse_data_row(line: str):
    s = line.strip()
    m = DATA_ROW_PATTERN_NEW.match(s)
    if m:
        g = m.groups()
        return {
            "roll": float(g[0]),
            "pitch": float(g[1]),
            "yaw": float(g[2]),
            "omega_r": float(g[3]),
            "omega_p": float(g[4]),
            "omega_y": float(g[5]),
            "tau_r": float(g[6]),
            "tau_p": float(g[7]),
            "tau_y": float(g[8]),
            "n0": int(g[9]),
            "n1": int(g[10]),
            "n2": int(g[11]),
            "v1": float(g[12]),
            "v2": float(g[13]),
            "v3": float(g[14]),
            "v1_des": float(g[15]),
            "v2_des": float(g[16]),
            "v3_des": float(g[17]),
            "e1": float(g[18]),
            "e2": float(g[19]),
            "e3": float(g[20]),
            "p1": float(g[21]),
            "p2": float(g[22]),
            "p3": float(g[23]),
        }
    m = DATA_ROW_PATTERN_OLD.match(s)
    if m:
        g = m.groups()
        v1, v2, v3 = float(g[12]), float(g[13]), float(g[14])
        return {
            "roll": float(g[0]),
            "pitch": float(g[1]),
            "yaw": float(g[2]),
            "omega_r": float(g[3]),
            "omega_p": float(g[4]),
            "omega_y": float(g[5]),
            "tau_r": float(g[6]),
            "tau_p": float(g[7]),
            "tau_y": float(g[8]),
            "n0": int(g[9]),
            "n1": int(g[10]),
            "n2": int(g[11]),
            "v1": v1,
            "v2": v2,
            "v3": v3,
            "v1_des": v1,
            "v2_des": v2,
            "v3_des": v3,
            "e1": float(g[15]),
            "e2": float(g[16]),
            "e3": float(g[17]),
            "p1": float(g[18]),
            "p2": float(g[19]),
            "p3": float(g[20]),
        }
    return None


def _serial_reader_thread(ser: serial.Serial, line_queue: queue.Queue, stop_event: threading.Event):
    """Read serial lines and put them in the queue; run in background thread."""
    try:
        while not stop_event.is_set():
            line = ser.readline()
            if not line:
                continue
            try:
                text = line.decode("utf-8", errors="replace").rstrip("\r\n")
            except Exception:
                continue
            if text:
                line_queue.put(("line", text))
    except Exception as e:
        line_queue.put(("error", str(e)))
    finally:
        line_queue.put(("done", None))


# All plottable/displayable keys and their display labels
SIGNAL_GROUPS = [
    ("Orientation (deg)", ["roll", "pitch", "yaw"]),
    ("Rate ω (rad/s)", ["omega_r", "omega_p", "omega_y"]),
    ("Torque τ", ["tau_r", "tau_p", "tau_y"]),
    ("Nodes", ["n0", "n1", "n2"]),
    ("Vel cmd (rev/s)", ["v1", "v2", "v3"]),
    ("Vel des (rev/s)", ["v1_des", "v2_des", "v3_des"]),
    ("Encoder", ["e1", "e2", "e3"]),
    ("Motor pos (rev)", ["p1", "p2", "p3"]),
    ("Tuning", TUNE_KEYS),
]
SIGNAL_LABELS = {
    "roll": "Roll", "pitch": "Pitch", "yaw": "Yaw",
    "omega_r": "ω_R", "omega_p": "ω_P", "omega_y": "ω_Y",
    "tau_r": "τ_R", "tau_p": "τ_P", "tau_y": "τ_Y",
    "n0": "n0", "n1": "n1", "n2": "n2",
    "v1": "v1", "v2": "v2", "v3": "v3",
    "v1_des": "v1_des", "v2_des": "v2_des", "v3_des": "v3_des",
    "e1": "e1", "e2": "e2", "e3": "e3",
    "p1": "p1", "p2": "p2", "p3": "p3",
    "M": "M", "J": "J", "H": "H", "X": "X",
    "Qa": "Qa", "Qr": "Qr", "R": "R",
    "KPh": "KPh", "KP": "KP", "KI": "KI", "KD": "KD",
}
ALL_KEYS = [k for _group, keys in SIGNAL_GROUPS for k in keys]
KEY_TO_GROUP = {k: group for group, keys in SIGNAL_GROUPS for k in keys}
PLOT_HISTORY_LEN = 800
VALUE_HISTORY_LEN = 20


def run(port: str, baud: int, csv_path: str | None, use_gui: bool = True):
    if use_gui:
        run_gui(port, baud, csv_path)
        return

    # Terminal-only mode
    ser = serial.Serial(port, baud, timeout=0.1)
    csv_file = None
    csv_header_written = False

    if csv_path:
        csv_file = open(csv_path, "w", newline="")

    last_tune = {}
    try:
        while True:
            line = ser.readline()
            if not line:
                continue
            try:
                text = line.decode("utf-8", errors="replace").rstrip("\r\n")
            except Exception:
                continue
            if not text:
                continue

            print(text)

            tune_parsed = parse_tune_line(text)
            if tune_parsed is not None:
                last_tune.update(tune_parsed)

            if csv_file and text.strip():
                row = parse_data_row(text)
                if row is not None:
                    for k in TUNE_KEYS:
                        row[k] = last_tune.get(k, 0.0)
                    if not csv_header_written:
                        csv_file.write(
                            "roll,pitch,yaw,omega_r,omega_p,omega_y,"
                            "tau_r,tau_p,tau_y,n0,n1,n2,v1,v2,v3,v1_des,v2_des,v3_des,e1,e2,e3,p1,p2,p3,"
                            + ",".join(TUNE_KEYS) + "\n"
                        )
                        csv_header_written = True
                    csv_file.write(
                        f"{row['roll']},{row['pitch']},{row['yaw']},"
                        f"{row['omega_r']},{row['omega_p']},{row['omega_y']},"
                        f"{row['tau_r']},{row['tau_p']},{row['tau_y']},"
                        f"{row['n0']},{row['n1']},{row['n2']},"
                        f"{row['v1']},{row['v2']},{row['v3']},"
                        f"{row['v1_des']},{row['v2_des']},{row['v3_des']},"
                        f"{row['e1']},{row['e2']},{row['e3']},"
                        f"{row['p1']},{row['p2']},{row['p3']},"
                        f"{','.join(str(row.get(k, 0)) for k in TUNE_KEYS)}\n"
                    )
                    csv_file.flush()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if csv_file:
            csv_file.close()
            if csv_path:
                print(f"\nLogged data rows to {csv_path}", file=sys.stderr)


def run_gui(port: str, baud: int, csv_path: str | None):
    import time
    import tkinter as tk
    from tkinter import font as tkfont, ttk

    try:
        import matplotlib.pyplot
        HAS_MATPLOTLIB = True
    except ImportError:
        HAS_MATPLOTLIB = False

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"Cannot open {port}: {e}", file=sys.stderr)
        sys.exit(1)
    ser_ref = [ser]
    line_queue = queue.Queue()
    stop_event = threading.Event()
    csv_file = None
    csv_header_written = False
    time_offset = [None]
    last_row = {}
    last_tune = {}
    recording = [False]
    record_start_time = [None]
    record_data = {k: ([], []) for k in ALL_KEYS}

    # Vel spin: log encoder velocity feedback (e1,e2,e3) to CSV when Arduino prints start/stop messages.
    torque_spin_logging = [False]
    torque_spin_csv_file = [None]
    torque_spin_start_time = [None]
    # First-order LPF for e1,e2,e3 written to CSV [s]. None = no filter (use raw).
    ENCODER_VEL_LPF_TAU_S = 0.05
    torque_spin_lpf_state = [None]  # None or {"e1", "e2", "e3", "t"}

    if csv_path:
        csv_file = open(csv_path, "w", newline="")

    def on_closing():
        stop_event.set()
        if ser_ref[0] is not None:
            try:
                ser_ref[0].close()
            except Exception:
                pass
            ser_ref[0] = None
        if csv_file:
            csv_file.close()
        if torque_spin_csv_file[0] is not None:
            try:
                torque_spin_csv_file[0].close()
            except OSError:
                pass
            torque_spin_csv_file[0] = None
        torque_spin_logging[0] = False
        root.destroy()

    def try_reconnect():
        if stop_event.is_set():
            return
        try:
            new_ser = serial.Serial(port, baud, timeout=0.1)
            ser_ref[0] = new_ser
            time_offset[0] = None
            root.title(f"Serial Monitor — {port} @ {baud}")
            reader = threading.Thread(
                target=_serial_reader_thread,
                args=(new_ser, line_queue, stop_event),
                daemon=True,
            )
            reader.start()
            root.after(10, pump_queue)
        except serial.SerialException:
            root.after(1500, try_reconnect)

    root = tk.Tk()
    root.title(f"Serial Monitor — {port} @ {baud}")
    root.geometry("1100x620")
    root.minsize(700, 400)

    # --- Left: grouped toggles ---
    left = tk.Frame(root, width=200, padx=8, pady=8)
    left.pack(side=tk.LEFT, fill=tk.Y)
    left.pack_propagate(False)
    tk.Label(left, text="Values / Plot", font=("", 11, "bold")).pack(anchor=tk.W)
    tk.Label(left, text="Show groups in Current values", font=("", 9), fg="gray").pack(anchor=tk.W)
    ttk.Separator(left, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)
    btn_row = tk.Frame(left)
    btn_row.pack(anchor=tk.W, pady=(0, 4))
    group_show_vars = {group_name: tk.BooleanVar(value=True) for group_name, _ in SIGNAL_GROUPS}

    def _set_all(on: bool):
        for v in group_show_vars.values():
            v.set(on)
        schedule_update()

    tk.Button(btn_row, text="All", command=lambda: _set_all(True), font=("", 8)).pack(side=tk.LEFT, padx=1)
    tk.Button(btn_row, text="None", command=lambda: _set_all(False), font=("", 8)).pack(side=tk.LEFT, padx=1)

    groups_frame = tk.Frame(left)
    groups_frame.pack(anchor=tk.W, pady=(4, 2), fill=tk.X)

    for i, (group_name, _keys) in enumerate(SIGNAL_GROUPS):
        cb = tk.Checkbutton(
            groups_frame,
            text=group_name,
            variable=group_show_vars[group_name],
            command=lambda: schedule_update(),
            anchor=tk.W,
            padx=4,
        )
        cb.grid(row=i, column=0, sticky="w", padx=2, pady=1)

    update_after_id = [None]

    def schedule_update():
        if update_after_id[0] is not None:
            root.after_cancel(update_after_id[0])
        def do_update():
            update_display()
            update_after_id[0] = None
        update_after_id[0] = root.after(150, do_update)

    for v in group_show_vars.values():
        v.trace_add("write", lambda *a: schedule_update())

    ttk.Separator(left, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=8)

    # --- Right: values + plot + raw log ---
    right = tk.Frame(root, padx=8, pady=8)
    right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    mono = tkfont.Font(family="Menlo", size=10)
    if "Menlo" not in mono.actual("family"):
        mono = tkfont.Font(family="Courier", size=10)

    panel_height_lines = 8

    right.rowconfigure(0, weight=1)
    right.rowconfigure(2, weight=1)
    right.columnconfigure(0, weight=1)

    values_frame = tk.LabelFrame(right, text=" Current values ", font=("", 10, "bold"))
    values_frame.grid(row=0, column=0, sticky="nsew", pady=(0, 0))
    values_frame.columnconfigure(0, weight=1)
    values_frame.rowconfigure(0, weight=1)
    values_container = tk.Frame(values_frame)
    values_container.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
    values_container.columnconfigure(0, weight=1)
    values_container.rowconfigure(0, weight=1)
    values_yscroll = ttk.Scrollbar(values_container)
    values_xscroll = ttk.Scrollbar(values_container, orient=tk.HORIZONTAL)
    value_tree = ttk.Treeview(
        values_container,
        columns=ALL_KEYS,
        show="headings",
        height=panel_height_lines,
        yscrollcommand=values_yscroll.set,
        xscrollcommand=values_xscroll.set,
    )
    for k in ALL_KEYS:
        value_tree.heading(k, text=SIGNAL_LABELS.get(k, k))
        value_tree.column(k, width=72, minwidth=40, stretch=False)
    values_yscroll.config(command=value_tree.yview)
    values_xscroll.config(command=value_tree.xview)
    value_tree.grid(row=0, column=0, sticky="nsew")
    values_yscroll.grid(row=0, column=1, sticky="ns")
    values_xscroll.grid(row=1, column=0, sticky="ew")
    values_container.columnconfigure(0, weight=1)
    values_container.rowconfigure(0, weight=1)
    value_history_iids = deque()

    def _set_panel_height(n):
        nonlocal panel_height_lines
        panel_height_lines = n
        value_tree.config(height=n)
        log_text.config(height=n)

    def make_resize_handle(parent, get_height, set_height, min_lines, max_lines):
        strip = tk.Frame(parent, height=6, bg="#b0b0b0", cursor="sb_v_double_arrow")
        strip.pack(fill=tk.X, pady=2)
        strip.pack_propagate(False)
        drag_state = [None]

        def on_press(e):
            drag_state[0] = (e.y_root, get_height())

        def on_motion(e):
            if drag_state[0] is None:
                return
            y0, h0 = drag_state[0]
            delta_lines = round((e.y_root - y0) / 15)
            new_h = max(min_lines, min(max_lines, h0 + delta_lines))
            set_height(new_h)
            drag_state[0] = (e.y_root, new_h)

        def on_release(_):
            drag_state[0] = None

        strip.bind("<ButtonPress-1>", on_press)
        strip.bind("<B1-Motion>", on_motion)
        strip.bind("<ButtonRelease-1>", on_release)

    handle_between = tk.Frame(right)
    handle_between.grid(row=1, column=0, sticky="ew")

    make_resize_handle(
        handle_between,
        get_height=lambda: panel_height_lines,
        set_height=_set_panel_height,
        min_lines=3,
        max_lines=30,
    )

    log_frame = tk.LabelFrame(right, text=" Raw log (last lines) ", font=("", 9, "bold"))
    log_frame.grid(row=2, column=0, sticky="nsew", pady=2)
    log_frame.columnconfigure(0, weight=1)
    log_frame.rowconfigure(0, weight=1)
    log_text = tk.Text(log_frame, height=panel_height_lines, wrap=tk.WORD, font=mono, bg="#1a1a1a", fg="#c0c0c0", insertbackground="white")
    log_text.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
    raw_lines = []

    pos_zero = {"p1": 0.0, "p2": 0.0, "p3": 0.0}

    def toggle_record():
        record_btn.config(state=tk.DISABLED)
        try:
            if recording[0]:
                recording[0] = False
                record_btn.config(text="Record")
                save_recording()
            else:
                recording[0] = True
                record_start_time[0] = None
                for k in ALL_KEYS:
                    record_data[k][0].clear()
                    record_data[k][1].clear()
                record_btn.config(text="Stop")
        finally:
            record_btn.config(state=tk.NORMAL)

    record_btn = tk.Button(left, text="Record", command=toggle_record)
    record_btn.pack(pady=4)

    def save_recording():
        script_dir = os.path.dirname(os.path.abspath(__file__))
        out_dir = os.path.join(script_dir, "data")
        os.makedirs(out_dir, exist_ok=True)
        encoder_keys = ["e1", "e2", "e3"]
        vel_keys = ["v1", "v2", "v3"]
        t_enc = record_data["e1"][0]
        t_vel = record_data["v1"][0]
        if t_enc:
            with open(os.path.join(out_dir, "encoder.csv"), "w", newline="") as f:
                f.write("time,e1,e2,e3\n")
                for i, t in enumerate(t_enc):
                    vals = ",".join(str(record_data[k][1][i]) for k in encoder_keys)
                    f.write(f"{t},{vals}\n")
        if t_vel:
            with open(os.path.join(out_dir, "vel_commands.csv"), "w", newline="") as f:
                f.write("time,v1,v2,v3\n")
                for i, t in enumerate(t_vel):
                    vals = ",".join(str(record_data[k][1][i]) for k in vel_keys)
                    f.write(f"{t},{vals}\n")
        if not HAS_MATPLOTLIB:
            return
        import matplotlib.pyplot as plt
        colors = "C0 C1 C2".split()
        if t_enc:
            fig, ax = plt.subplots()
            for i, k in enumerate(encoder_keys):
                ax.plot(record_data[k][0], record_data[k][1], label=SIGNAL_LABELS.get(k, k), color=colors[i % len(colors)])
            ax.set_xlabel("Time (s)")
            ax.legend(loc="upper right", fontsize=8)
            ax.grid(True, alpha=0.4)
            fig.tight_layout()
            fig.savefig(os.path.join(out_dir, "encoder.png"), dpi=150)
            plt.close(fig)
        if t_vel:
            fig, ax = plt.subplots()
            for i, k in enumerate(vel_keys):
                ax.plot(record_data[k][0], record_data[k][1], label=SIGNAL_LABELS.get(k, k), color=colors[i % len(colors)])
            ax.set_xlabel("Time (s)")
            ax.legend(loc="upper right", fontsize=8)
            ax.grid(True, alpha=0.4)
            fig.tight_layout()
            fig.savefig(os.path.join(out_dir, "vel_commands.png"), dpi=150)
            plt.close(fig)

    def update_display():
        for k in ALL_KEYS:
            if group_show_vars[KEY_TO_GROUP[k]].get():
                value_tree.column(k, width=72, minwidth=40, stretch=False)
            else:
                value_tree.column(k, width=0, minwidth=0, stretch=False)

    def pump_queue():
        nonlocal csv_header_written
        try:
            while True:
                kind, payload = line_queue.get_nowait()
                if kind == "done":
                    if ser_ref[0] is not None:
                        try:
                            ser_ref[0].close()
                        except Exception:
                            pass
                        ser_ref[0] = None
                    root.title(f"Serial Monitor — {port} @ {baud} (reconnecting…)")
                    root.after(1500, try_reconnect)
                    return
                if kind == "error":
                    raw_lines.append(f"[Error] {payload}")
                    if len(raw_lines) > 50:
                        raw_lines.pop(0)
                    continue
                raw_lines.append(payload)
                if len(raw_lines) > 100:
                    raw_lines.pop(0)
                log_text.delete("1.0", tk.END)
                log_text.insert(tk.END, "\n".join(raw_lines[-20:]))
                log_text.see(tk.END)

                # Parse TUNE lines and keep last values for Current values / CSV
                tune_parsed = parse_tune_line(payload)
                if tune_parsed is not None:
                    last_tune.update(tune_parsed)

                # Start vel-spin CSV log: encoder velocity feedback (e1,e2,e3) [rev/s] during vel spin test.
                if "Vel spin: all motors at" in payload and not torque_spin_logging[0]:
                    torque_spin_logging[0] = True
                    torque_spin_lpf_state[0] = None
                    script_dir = os.path.dirname(os.path.abspath(__file__))
                    data_dir = os.path.join(script_dir, "data")
                    os.makedirs(data_dir, exist_ok=True)
                    try:
                        torque_spin_csv_file[0] = open(
                            os.path.join(data_dir, "encoder_velocities.csv"), "w", newline=""
                        )
                        torque_spin_csv_file[0].write("# Encoder velocity feedback e1,e2,e3 [rev/s] LPF tau=%.2fs\n" % ENCODER_VEL_LPF_TAU_S)
                        torque_spin_csv_file[0].write("time,e1,e2,e3\n")
                        torque_spin_csv_file[0].flush()
                        torque_spin_start_time[0] = time.perf_counter()
                    except OSError:
                        torque_spin_logging[0] = False
                # Stop vel-spin CSV log when Arduino prints the done message.
                if ("Vel spin done" in payload or "Resuming normal operations" in payload) and torque_spin_logging[0]:
                    torque_spin_logging[0] = False
                    torque_spin_lpf_state[0] = None
                    # Close encoder CSV only if not recording via Record button.
                    if not recording[0] and torque_spin_csv_file[0] is not None:
                        try:
                            torque_spin_csv_file[0].close()
                        except OSError:
                            pass
                        torque_spin_csv_file[0] = None

                # Log encoder vel (LPF e1,e2,e3) when vel-spin auto-logging is active (Record uses its own CSV).
                if torque_spin_logging[0] and torque_spin_csv_file[0] is not None and payload.strip():
                    row = parse_data_row(payload)
                    if row is not None:
                        t_elapsed = time.perf_counter() - torque_spin_start_time[0]
                        s = torque_spin_lpf_state[0]
                        tau = ENCODER_VEL_LPF_TAU_S
                        if s is None:
                            e1, e2, e3 = row["e1"], row["e2"], row["e3"]
                            torque_spin_lpf_state[0] = {"e1": e1, "e2": e2, "e3": e3, "t": t_elapsed}
                        else:
                            dt = t_elapsed - s["t"]
                            if dt <= 0 or dt > 1.0:
                                dt = 0.05
                            alpha = dt / (tau + dt)
                            e1 = s["e1"] + alpha * (row["e1"] - s["e1"])
                            e2 = s["e2"] + alpha * (row["e2"] - s["e2"])
                            e3 = s["e3"] + alpha * (row["e3"] - s["e3"])
                            torque_spin_lpf_state[0] = {"e1": e1, "e2": e2, "e3": e3, "t": t_elapsed}
                        torque_spin_csv_file[0].write(
                            f"{t_elapsed:.3f},{e1:.3f},{e2:.3f},{e3:.3f}\n"
                        )
                        torque_spin_csv_file[0].flush()

                row = parse_data_row(payload)
                if row is not None:
                    for k in TUNE_KEYS:
                        row[k] = last_tune.get(k, 0.0)
                if csv_file and payload.strip() and row is not None:
                    if not csv_header_written:
                        csv_file.write("roll,pitch,yaw,omega_r,omega_p,omega_y,tau_r,tau_p,tau_y,n0,n1,n2,v1,v2,v3,v1_des,v2_des,v3_des,e1,e2,e3,p1,p2,p3," + ",".join(TUNE_KEYS) + "\n")
                        csv_header_written = True
                    csv_file.write(f"{row['roll']},{row['pitch']},{row['yaw']},{row['omega_r']},{row['omega_p']},{row['omega_y']},{row['tau_r']},{row['tau_p']},{row['tau_y']},{row['n0']},{row['n1']},{row['n2']},{row['v1']},{row['v2']},{row['v3']},{row['v1_des']},{row['v2_des']},{row['v3_des']},{row['e1']},{row['e2']},{row['e3']},{row['p1']},{row['p2']},{row['p3']}," + ",".join(str(row.get(k, 0)) for k in TUNE_KEYS) + "\n")
                    csv_file.flush()
                if row is not None:
                    t = time.perf_counter()
                    if time_offset[0] is None:
                        time_offset[0] = t
                    t_rel = t - time_offset[0]
                    for k in ALL_KEYS:
                        last_row[k] = row[k]
                    if recording[0]:
                        if record_start_time[0] is None:
                            record_start_time[0] = t_rel
                        t_rec = t_rel - record_start_time[0]
                        for k in ALL_KEYS:
                            val = row[k] - pos_zero.get(k, 0.0) if k in pos_zero else row[k]
                            record_data[k][0].append(t_rec)
                            record_data[k][1].append(val)
                    disp_row = dict(row)
                    for pk in ("p1", "p2", "p3"):
                        disp_row[pk] = row[pk] - pos_zero[pk]
                    vals_str = [
                        f"{disp_row[k]:.4f}" if isinstance(disp_row[k], float) else str(disp_row[k])
                        for k in ALL_KEYS
                    ]
                    iid = value_tree.insert("", tk.END, values=vals_str)
                    value_history_iids.append(iid)
                    while len(value_history_iids) > VALUE_HISTORY_LEN:
                        old_iid = value_history_iids.popleft()
                        try:
                            value_tree.delete(old_iid)
                        except tk.TclError:
                            pass
        except queue.Empty:
            pass
        root.after(10, pump_queue)

    root.after(0, update_display)
    reader = threading.Thread(target=_serial_reader_thread, args=(ser_ref[0], line_queue, stop_event), daemon=True)
    reader.start()
    root.after(10, pump_queue)
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


def main():
    ap = argparse.ArgumentParser(
        description="Read Arduino serial and display printouts (including encoder e1,e2,e3)."
    )
    ap.add_argument("--port", "-p", type=str, default=None, help="Serial port (e.g. /dev/ttyACM0 or COM3)")
    ap.add_argument("--list", "-l", action="store_true", help="List available serial ports and exit")
    ap.add_argument("--no-gui", action="store_true", help="Stream to terminal only (no GUI window)")
    ap.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUD, help=f"Baud rate (default {DEFAULT_BAUD})")
    ap.add_argument("--csv", type=str, default=None, metavar="FILE", help="Log data rows (with encoder columns) to CSV file")
    args = ap.parse_args()

    if args.list:
        print("Serial ports:")
        list_ports()
        return

    port = args.port or find_likely_port()
    if not port:
        print("No serial port found. Use --port or --list.", file=sys.stderr)
        sys.exit(1)
    if not args.port:
        print(f"Using port: {port}", file=sys.stderr)

    run(port, args.baud, args.csv, use_gui=not args.no_gui)


if __name__ == "__main__":
    main()
