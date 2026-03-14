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
import queue
import re
import sys
import threading

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Install pyserial: pip install pyserial", file=sys.stderr)
    sys.exit(1)

# Arduino Serial.begin(115200)
DEFAULT_BAUD = 115200

# Data row: space-separated floats/ints; header line contains "omega_R" or "Roll"
# Format: Roll Pitch Yaw | omega_R omega_P omega_Y | tau_R tau_P tau_Y | n0 n1 n2 | v1 v2 v3 | e1 e2 e3 | scale max_vel
DATA_ROW_PATTERN = re.compile(
    r"^\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+\|\s*"   # roll pitch yaw
    r"([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+\|\s*"       # omega_r omega_p omega_y
    r"([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+\|\s*"       # tau_r tau_p tau_y
    r"([01])\s+([01])\s+([01])\s+\|\s*"                 # n0 n1 n2
    r"([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+\|\s*"       # v1 v2 v3
    r"([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+\|\s*"       # e1 e2 e3
    r"([-\d.]+)\s+([-\d.]+)\s*$"                       # scale max_vel
)


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
    m = DATA_ROW_PATTERN.match(line.strip())
    if not m:
        return None
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
        "e1": float(g[15]),
        "e2": float(g[16]),
        "e3": float(g[17]),
        "scale": float(g[18]),
        "max_vel": float(g[19]),
    }


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
    ("Vel cmd", ["v1", "v2", "v3"]),
    ("Encoder", ["e1", "e2", "e3"]),
    ("Other", ["scale", "max_vel"]),
]
SIGNAL_LABELS = {
    "roll": "Roll", "pitch": "Pitch", "yaw": "Yaw",
    "omega_r": "ω_R", "omega_p": "ω_P", "omega_y": "ω_Y",
    "tau_r": "τ_R", "tau_p": "τ_P", "tau_y": "τ_Y",
    "n0": "n0", "n1": "n1", "n2": "n2",
    "v1": "v1", "v2": "v2", "v3": "v3",
    "e1": "e1", "e2": "e2", "e3": "e3",
    "scale": "scale", "max_vel": "max_vel",
}
ALL_KEYS = [k for _group, keys in SIGNAL_GROUPS for k in keys]
PLOT_HISTORY_LEN = 800


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

            if csv_file and text.strip():
                row = parse_data_row(text)
                if row is not None:
                    if not csv_header_written:
                        csv_file.write(
                            "roll,pitch,yaw,omega_r,omega_p,omega_y,"
                            "tau_r,tau_p,tau_y,n0,n1,n2,v1,v2,v3,e1,e2,e3,scale,max_vel\n"
                        )
                        csv_header_written = True
                    csv_file.write(
                        f"{row['roll']},{row['pitch']},{row['yaw']},"
                        f"{row['omega_r']},{row['omega_p']},{row['omega_y']},"
                        f"{row['tau_r']},{row['tau_p']},{row['tau_y']},"
                        f"{row['n0']},{row['n1']},{row['n2']},"
                        f"{row['v1']},{row['v2']},{row['v3']},"
                        f"{row['e1']},{row['e2']},{row['e3']},"
                        f"{row['scale']},{row['max_vel']}\n"
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
        import matplotlib
        matplotlib.use("TkAgg")
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        from matplotlib.figure import Figure
        HAS_MATPLOTLIB = True
    except ImportError:
        HAS_MATPLOTLIB = False

    ser = serial.Serial(port, baud, timeout=0.1)
    line_queue = queue.Queue()
    stop_event = threading.Event()
    csv_file = None
    csv_header_written = False
    last_row = {}
    plot_data = {k: ([], []) for k in ALL_KEYS}
    time_offset = [None]

    if csv_path:
        csv_file = open(csv_path, "w", newline="")

    def on_closing():
        stop_event.set()
        ser.close()
        if csv_file:
            csv_file.close()
        root.destroy()

    root = tk.Tk()
    root.title(f"Serial Monitor — {port} @ {baud}")
    root.geometry("1100x620")
    root.minsize(700, 400)

    # --- Left: checkboxes ---
    left = tk.Frame(root, width=200, padx=8, pady=8)
    left.pack(side=tk.LEFT, fill=tk.Y)
    left.pack_propagate(False)
    tk.Label(left, text="Show / Plot", font=("", 11, "bold")).pack(anchor=tk.W)
    tk.Label(left, text="Check to display and plot", font=("", 9), fg="gray").pack(anchor=tk.W)
    ttk.Separator(left, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=6)

    cb_vars = {}
    for group_name, keys in SIGNAL_GROUPS:
        tk.Label(left, text=group_name, font=("", 9, "bold")).pack(anchor=tk.W, pady=(8, 2))
        for k in keys:
            v = tk.BooleanVar(value=(k in ("e1", "e2", "e3")))
            tk.Checkbutton(left, text=SIGNAL_LABELS.get(k, k), variable=v, anchor=tk.W).pack(anchor=tk.W, padx=12)
            cb_vars[k] = v
        if group_name == "Encoder":
            f = tk.Frame(left)
            f.pack(anchor=tk.W, padx=12)
            tk.Button(f, text="All", command=lambda: [cb_vars[x].set(True) for x in ("e1", "e2", "e3")], font=("", 8)).pack(side=tk.LEFT, padx=1)
            tk.Button(f, text="None", command=lambda: [cb_vars[x].set(False) for x in ("e1", "e2", "e3")], font=("", 8)).pack(side=tk.LEFT, padx=1)

    for v in cb_vars.values():
        v.trace_add("write", lambda *a: root.after(50, update_display_and_plot))

    ttk.Separator(left, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=8)

    # --- Right: values + plot + raw log ---
    right = tk.Frame(root, padx=8, pady=8)
    right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    values_frame = tk.LabelFrame(right, text=" Current values ", font=("", 10, "bold"))
    values_frame.pack(fill=tk.X, pady=(0, 6))
    values_inner = tk.Frame(values_frame)
    values_inner.pack(fill=tk.X, padx=6, pady=4)
    value_labels = {}

    mono = tkfont.Font(family="Menlo", size=10)
    if "Menlo" not in mono.actual("family"):
        mono = tkfont.Font(family="Courier", size=10)

    for _group_name, keys in SIGNAL_GROUPS:
        for k in keys:
            row_f = tk.Frame(values_inner)
            row_f.pack(fill=tk.X)
            lbl = tk.Label(row_f, text=f"{SIGNAL_LABELS.get(k, k)}:", font=mono, width=10, anchor=tk.E)
            lbl.pack(side=tk.LEFT, padx=2)
            val_lbl = tk.Label(row_f, text="—", font=mono, width=12, anchor=tk.W, fg="#0a0")
            val_lbl.pack(side=tk.LEFT, padx=2)
            value_labels[k] = (row_f, lbl, val_lbl)

    plot_frame = tk.LabelFrame(right, text=" Plot ", font=("", 10, "bold"))
    plot_frame.pack(fill=tk.BOTH, expand=True, pady=6)

    if HAS_MATPLOTLIB:
        fig = Figure(figsize=(6, 3), dpi=100)
        ax = fig.add_subplot(111)
        ax.set_facecolor("#f8f8f8")
        fig.patch.set_facecolor("#f0f0f0")
        canvas = FigureCanvasTkAgg(fig, master=plot_frame)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    else:
        tk.Label(plot_frame, text="Install matplotlib for plotting: pip install matplotlib", fg="gray").pack(expand=True)

    log_frame = tk.LabelFrame(right, text=" Raw log (last lines) ", font=("", 9, "bold"))
    log_frame.pack(fill=tk.X, pady=4)
    log_text = tk.Text(log_frame, height=3, wrap=tk.WORD, font=mono, bg="#1a1a1a", fg="#c0c0c0", insertbackground="white")
    log_text.pack(fill=tk.X, padx=4, pady=4)
    raw_lines = []

    def clear_plot():
        for k in ALL_KEYS:
            plot_data[k][0].clear()
            plot_data[k][1].clear()
        time_offset[0] = None
        if HAS_MATPLOTLIB:
            ax.clear()
            canvas.draw_idle()

    tk.Button(left, text="Clear plot", command=clear_plot).pack(pady=4)

    def update_display_and_plot():
        for k in ALL_KEYS:
            row_f, lbl, val_lbl = value_labels[k]
            if cb_vars[k].get():
                row_f.pack(fill=tk.X)
                v = last_row.get(k)
                if v is not None:
                    val_lbl.config(text=f"{v:.4f}" if isinstance(v, float) else str(v))
                else:
                    val_lbl.config(text="—")
            else:
                row_f.pack_forget()
        if not HAS_MATPLOTLIB:
            return
        ax.clear()
        colors = "C0 C1 C2 C3 C4 C5 C6 C7 C8 C9".split()
        for i, k in enumerate(ALL_KEYS):
            if not cb_vars[k].get():
                continue
            t, y = plot_data[k]
            if not t:
                continue
            ax.plot(t, y, label=SIGNAL_LABELS.get(k, k), color=colors[i % len(colors)])
        ax.set_xlabel("Time (s)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.4)
        ax.set_facecolor("#f8f8f8")
        fig.tight_layout()
        canvas.draw_idle()

    def pump_queue():
        nonlocal csv_header_written
        try:
            while True:
                kind, payload = line_queue.get_nowait()
                if kind == "done":
                    root.after(100, pump_queue)
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
                if csv_file and payload.strip():
                    row = parse_data_row(payload)
                    if row is not None:
                        if not csv_header_written:
                            csv_file.write("roll,pitch,yaw,omega_r,omega_p,omega_y,tau_r,tau_p,tau_y,n0,n1,n2,v1,v2,v3,e1,e2,e3,scale,max_vel\n")
                            csv_header_written = True
                        csv_file.write(f"{row['roll']},{row['pitch']},{row['yaw']},{row['omega_r']},{row['omega_p']},{row['omega_y']},{row['tau_r']},{row['tau_p']},{row['tau_y']},{row['n0']},{row['n1']},{row['n2']},{row['v1']},{row['v2']},{row['v3']},{row['e1']},{row['e2']},{row['e3']},{row['scale']},{row['max_vel']}\n")
                        csv_file.flush()
                row = parse_data_row(payload)
                if row is not None:
                    t = time.perf_counter()
                    if time_offset[0] is None:
                        time_offset[0] = t
                    t_rel = t - time_offset[0]
                    for k in ALL_KEYS:
                        last_row[k] = row[k]
                        times, vals = plot_data[k]
                        times.append(t_rel)
                        vals.append(row[k])
                        if len(times) > PLOT_HISTORY_LEN:
                            times.pop(0)
                            vals.pop(0)
                    root.after(0, update_display_and_plot)
        except queue.Empty:
            pass
        root.after(50, pump_queue)

    root.after(0, update_display_and_plot)
    reader = threading.Thread(target=_serial_reader_thread, args=(ser, line_queue, stop_event), daemon=True)
    reader.start()
    root.after(50, pump_queue)
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
