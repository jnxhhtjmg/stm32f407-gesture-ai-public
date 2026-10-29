#!/usr/bin/env python3
"""Host GUI for the STM32F407 gesture terminal.

Features
--------
* connect to the board over USART1 (115200 8N1)
* show the current gesture, confidence and inference time
* start/stop CSV capture and save one file per capture
* parse the same text protocol as training/collect_f407.py

PC -> MCU frame:  AA 55 FUNC LEN PAYLOAD CRC16
CTRL (0x87):      0x01 = capture start, 0x02 = capture stop
"""
from __future__ import annotations

import argparse
import queue
import re
import threading
import time
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import serial
from serial.tools import list_ports

BAUD = 115200
BG = "#0b1117"
PANEL = "#121b24"
PANEL_2 = "#172330"
FG = "#e6edf3"
MUTED = "#8b9bab"
ACCENT = "#4fd1c5"
ACCENT_DARK = "#2c9f95"
WARN = "#f6ad55"
DANGER = "#fc8181"

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_CAPTURE_DIR = SCRIPT_DIR / "data" / "gui_capture"

GESTURE_TEXT = {
    "lift": "LIFT",
    "move_left": "MOVE LEFT",
    "move_right": "MOVE RIGHT",
    "neutral": "NEUTRAL",
    "tilt_left": "TILT LEFT",
    "tilt_right": "TILT RIGHT",
}


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def make_frame(func: int, payload: bytes = b"") -> bytes:
    body = bytes((0xAA, 0x55, func, len(payload))) + payload
    crc = crc16_modbus(body)
    return body + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


class SerialWorker(threading.Thread):
    def __init__(self, port: str, baud: int, events: queue.Queue):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.events = events
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.ser: serial.Serial | None = None

    def run(self) -> None:
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        except Exception as exc:  # noqa: BLE001
            self.events.put(("error", f"cannot open {self.port}: {exc}"))
            self.events.put(("disconnected", self.port))
            return
        self.events.put(("connected", self.port))
        try:
            while not self._stop.is_set():
                try:
                    raw = self.ser.readline()
                except Exception as exc:  # noqa: BLE001
                    self.events.put(("error", f"serial read failed: {exc}"))
                    break
                if not raw:
                    continue
                text = raw.decode("utf-8", "replace").strip("\r\n")
                if text:
                    self.events.put(("line", text))
        finally:
            try:
                self.ser.close()
            except Exception:  # noqa: BLE001
                pass
            self.events.put(("disconnected", self.port))

    def send(self, data: bytes) -> bool:
        with self._lock:
            if self.ser is not None and self.ser.is_open:
                try:
                    self.ser.write(data)
                    self.ser.flush()
                    return True
                except Exception as exc:  # noqa: BLE001
                    self.events.put(("error", f"serial write failed: {exc}"))
                    return False
        return False

    def stop(self) -> None:
        self._stop.set()


class GestureApp:
    def __init__(self, root: tk.Tk, demo: bool = False) -> None:
        self.root = root
        self.demo = demo
        self.events: queue.Queue = queue.Queue()
        self.worker: SerialWorker | None = None
        self.capture_file = None
        self.capture_rows = 0
        self.capture_label = ""
        self.capture_path: Path | None = None
        self._last_imu = ""
        self._last_infer_us = 0
        self._samples = 0
        self._connected = False
        self._start_time = time.time()
        self._demo_index = 0

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(BAUD))
        self.label_var = tk.StringVar(value="lift")
        self.dir_var = tk.StringVar(value=str(DEFAULT_CAPTURE_DIR))
        self.status_var = tk.StringVar(value="OFFLINE")
        self.gesture_var = tk.StringVar(value="--")
        self.conf_var = tk.StringVar(value="confidence --%")
        self.infer_var = tk.StringVar(value="infer  -- ms")
        self.sample_var = tk.StringVar(value="samples 0")
        self.file_var = tk.StringVar(value="no capture file")
        self._build()
        self._refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(50, self._poll_events)
        if demo:
            self.port_var.set("DEMO")
            self.status_var.set("DEMO MODE")
            self._set_connected_ui(True, demo=True)
            self._demo_tick()

    # ------------------------------------------------------------------ UI
    def _build(self) -> None:
        self.root.title("F407 Gesture Monitor")
        self.root.geometry("1120x720")
        self.root.minsize(980, 640)
        self.root.configure(bg=BG)

        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", font=("Segoe UI", 10))
        style.configure("TFrame", background=BG)
        style.configure("Panel.TFrame", background=PANEL)
        style.configure("TLabel", background=BG, foreground=FG)
        style.configure("Panel.TLabel", background=PANEL, foreground=FG)
        style.configure("Muted.TLabel", background=BG, foreground=MUTED)
        style.configure("PanelMuted.TLabel", background=PANEL, foreground=MUTED)
        style.configure("Accent.TLabel", background=BG, foreground=ACCENT)
        style.configure("Title.TLabel", background=BG, foreground=FG,
                        font=("Segoe UI Semibold", 18))
        style.configure("TButton", background=PANEL_2, foreground=FG,
                        borderwidth=0, padding=(12, 7))
        style.map("TButton", background=[("active", ACCENT_DARK)],
                  foreground=[("disabled", MUTED)])
        style.configure("Accent.TButton", background=ACCENT, foreground="#071014",
                        font=("Segoe UI Semibold", 10))
        style.map("Accent.TButton", background=[("active", ACCENT_DARK)])
        style.configure("TEntry", fieldbackground=PANEL_2, foreground=FG,
                        insertcolor=FG, borderwidth=0, padding=6)
        style.configure("TCombobox", fieldbackground=PANEL_2, foreground=FG,
                        background=PANEL_2, arrowcolor=FG, borderwidth=0)
        style.map("TCombobox", fieldbackground=[("readonly", PANEL_2)],
                  foreground=[("readonly", FG)])
        style.configure("Accent.Horizontal.TProgressbar", troughcolor=PANEL_2,
                        background=ACCENT, borderwidth=0, thickness=14)
        style.configure("TSeparator", background=PANEL_2)

        header = ttk.Frame(self.root, padding=(22, 16, 22, 8))
        header.pack(fill="x")
        ttk.Label(header, text="F407 GESTURE MONITOR", style="Title.TLabel").pack(side="left")
        ttk.Label(header, text="  MPU6050 · 100 Hz · INT8 1D-CNN · FreeRTOS",
                  style="Muted.TLabel").pack(side="left", pady=(4, 0))

        conn = ttk.Frame(self.root, padding=(22, 0, 22, 10))
        conn.pack(fill="x")
        ttk.Label(conn, text="PORT", style="Muted.TLabel").pack(side="left")
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=12,
                                       state="readonly")
        self.port_combo.pack(side="left", padx=(6, 10))
        ttk.Label(conn, text="BAUD", style="Muted.TLabel").pack(side="left")
        self.baud_combo = ttk.Combobox(conn, textvariable=self.baud_var, width=8,
                                       state="readonly",
                                       values=("115200", "460800", "921600"))
        self.baud_combo.pack(side="left", padx=(6, 10))
        self.connect_btn = ttk.Button(conn, text="CONNECT", style="Accent.TButton",
                                      command=self.toggle_connection)
        self.connect_btn.pack(side="left")
        self.status_label = ttk.Label(conn, textvariable=self.status_var,
                                      style="Accent.TLabel")
        self.status_label.pack(side="right")

        body = ttk.Frame(self.root, padding=(22, 4, 22, 8))
        body.pack(fill="both", expand=True)
        body.columnconfigure(0, weight=3, minsize=520)
        body.columnconfigure(1, weight=2, minsize=340)
        body.rowconfigure(0, weight=1)

        left = ttk.Frame(body, style="Panel.TFrame", padding=22)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        ttk.Label(left, text="CURRENT GESTURE", style="PanelMuted.TLabel").pack(anchor="w")
        self.gesture_label = tk.Label(left, textvariable=self.gesture_var,
                                      bg=PANEL, fg=ACCENT,
                                      font=("Segoe UI Semibold", 44))
        self.gesture_label.pack(anchor="w", pady=(12, 0))
        self.conf_label = ttk.Label(left, textvariable=self.conf_var,
                                    style="Panel.TLabel", font=("Segoe UI", 12),
                                    anchor="w")
        self.conf_label.pack(anchor="w", pady=(4, 10))
        self.conf_bar = ttk.Progressbar(left, maximum=100, value=0,
                                        style="Accent.Horizontal.TProgressbar")
        self.conf_bar.pack(fill="x")
        self.infer_label = ttk.Label(left, textvariable=self.infer_var,
                                     style="PanelMuted.TLabel", font=("Consolas", 12))
        self.infer_label.pack(anchor="w", pady=(18, 0))
        self.sample_label = ttk.Label(left, textvariable=self.sample_var,
                                      style="PanelMuted.TLabel", font=("Consolas", 12))
        self.sample_label.pack(anchor="w", pady=(4, 0))
        self.file_label = ttk.Label(left, textvariable=self.file_var,
                                    style="PanelMuted.TLabel", wraplength=450)
        self.file_label.pack(anchor="w", pady=(18, 0))
        ttk.Label(left, text="Tip: press KEY0 on the board to start/stop a capture.",
                  style="PanelMuted.TLabel").pack(anchor="w", pady=(14, 0))

        right = ttk.Frame(body, style="Panel.TFrame", padding=18)
        right.grid(row=0, column=1, sticky="nsew")
        ttk.Label(right, text="CAPTURE", style="PanelMuted.TLabel").pack(anchor="w")
        ttk.Label(right, text="LABEL", style="PanelMuted.TLabel").pack(anchor="w", pady=(10, 2))
        ttk.Combobox(right, textvariable=self.label_var, state="normal",
                     values=("lift", "move_left", "move_right", "neutral",
                             "tilt_left", "tilt_right")).pack(fill="x")
        ttk.Label(right, text="OUTPUT DIR", style="PanelMuted.TLabel").pack(anchor="w", pady=(10, 2))
        dir_row = ttk.Frame(right, style="Panel.TFrame")
        dir_row.pack(fill="x")
        ttk.Entry(dir_row, textvariable=self.dir_var).pack(side="left", fill="x", expand=True)
        ttk.Button(dir_row, text="...", width=4, command=self.choose_dir).pack(side="left", padx=(6, 0))
        btn_row = ttk.Frame(right, style="Panel.TFrame")
        btn_row.pack(fill="x", pady=(14, 0))
        self.start_btn = ttk.Button(btn_row, text="START", style="Accent.TButton",
                                    state="disabled",
                                    command=lambda: self.send_ctrl(0x01))
        self.start_btn.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.stop_btn = ttk.Button(btn_row, text="STOP", state="disabled",
                                   command=lambda: self.send_ctrl(0x02))
        self.stop_btn.pack(side="left", fill="x", expand=True, padx=(4, 0))

        ttk.Separator(right).pack(fill="x", pady=16)
        ttk.Label(right, text="QUICK ACTIONS", style="PanelMuted.TLabel").pack(anchor="w")
        act = ttk.Frame(right, style="Panel.TFrame")
        act.pack(fill="x", pady=(8, 0))
        ttk.Button(act, text="STATUS", command=lambda: self.send_frame(make_frame(0x81))).pack(
            side="left", fill="x", expand=True, padx=(0, 4))
        ttk.Button(act, text="CLEAR LOG", command=self.clear_log).pack(
            side="left", fill="x", expand=True, padx=(4, 0))

        log_frame = ttk.Frame(self.root, padding=(22, 0, 22, 14))
        log_frame.pack(fill="both", expand=False)
        ttk.Label(log_frame, text="SERIAL LOG", style="Muted.TLabel").pack(anchor="w")
        self.log = tk.Text(log_frame, height=9, bg=PANEL, fg=FG, insertbackground=FG,
                           relief="flat", font=("Consolas", 9), wrap="none")
        self.log.pack(side="left", fill="both", expand=True, pady=(4, 0))
        scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        scroll.pack(side="right", fill="y", pady=(4, 0))
        self.log.configure(yscrollcommand=scroll.set, state="disabled")

    # -------------------------------------------------------------- serial
    def _refresh_ports(self) -> None:
        ports = [p.device for p in list_ports.comports()]
        if "COM6" not in ports:
            ports.insert(0, "COM6")
        self.port_combo["values"] = ports
        if not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_connection(self) -> None:
        if self.demo:
            return
        if self.worker is not None:
            self.disconnect()
        else:
            self.connect()

    def connect(self) -> None:
        if self.worker is not None:
            return
        port = self.port_var.get().strip()
        if not port:
            return
        self.worker = SerialWorker(port, int(self.baud_var.get()), self.events)
        self.worker.start()
        self.status_var.set(f"connecting {port} ...")
        self.connect_btn.configure(text="DISCONNECT")
        self._log(f"connecting {port} @ {self.baud_var.get()}")

    def disconnect(self) -> None:
        worker = self.worker
        self.worker = None
        if worker is not None:
            worker.stop()
        self._connected = False
        self._set_connected_ui(False)
        self.status_var.set("OFFLINE")
        self.connect_btn.configure(text="CONNECT")
        self._log("disconnected")

    def _set_connected_ui(self, connected: bool, demo: bool = False) -> None:
        self._connected = connected
        state = "normal" if connected else "disabled"
        self.start_btn.configure(state=state)
        self.stop_btn.configure(state=state)
        self.port_combo.configure(state="disabled" if connected and not demo else "readonly")
        self.baud_combo.configure(state="disabled" if connected else "readonly")
        if demo:
            self.connect_btn.configure(state="disabled")
        if not connected:
            self.start_btn.configure(state="disabled")
            self.stop_btn.configure(state="disabled")

    def send_frame(self, frame: bytes) -> None:
        if self.demo:
            self._log("demo: " + frame.hex(" "))
            return
        if self.worker is None or not self.worker.send(frame):
            self._log("send failed: not connected")
            return
        self._log("tx " + frame.hex(" "))

    def send_ctrl(self, command: int) -> None:
        self.send_frame(make_frame(0x87, bytes((command,))))
        if command == 0x01:
            self._log(f"capture start requested (label={self.label_var.get().strip() or 'unlabeled'})")
        else:
            self._log("capture stop requested")

    # ------------------------------------------------------------- events
    def _poll_events(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "line":
                    self._handle_line(payload)
                elif kind == "connected":
                    self._connected = True
                    self.status_var.set(f"ONLINE {payload}")
                    self._set_connected_ui(True)
                    self._log(f"connected {payload}")
                elif kind == "disconnected":
                    self._connected = False
                    self.worker = None
                    self.connect_btn.configure(text="CONNECT", state="normal")
                    self._set_connected_ui(False)
                    self.status_var.set("OFFLINE")
                    self._log("serial closed")
                elif kind == "error":
                    self._log("ERROR " + str(payload))
                    self.status_var.set("ERROR")
        except queue.Empty:
            pass
        self.root.after(50, self._poll_events)

    def _handle_line(self, line: str) -> None:
        if line.startswith("gesture,"):
            parts = line.split(",")
            if len(parts) >= 3:
                label = parts[1].strip()
                try:
                    conf = int(float(parts[2]))
                except ValueError:
                    conf = 0
                self._show_gesture(label, conf)
            return
        if line.startswith("infer_us,"):
            try:
                self._last_infer_us = int(line.split(",", 1)[1])
            except ValueError:
                return
            self.infer_var.set(f"infer  {self._last_infer_us / 1000.0:.1f} ms")
            return
        if line.startswith("imu,"):
            self._samples += 1
            self._last_imu = line
            if self._samples % 10 == 0:
                self.sample_var.set(f"samples {self._samples}")
            return
        if line.startswith("capture,start"):
            self._log(line)
            self.open_capture()
            return
        if line.startswith("capture,stop"):
            self._log(line)
            self.close_capture()
            return
        if re.fullmatch(r"-?\d+(?:,-?\d+){6}", line):
            self.write_row(line)
            return
        if line.startswith(("event,", "sampling,", "clock,", "stack_free,",
                            "bench_", "oled", "gyro", "MPU6050", "f407_")):
            self._log(line)
            return
        self._log(line)

    def _show_gesture(self, label: str, conf: int) -> None:
        text = GESTURE_TEXT.get(label, label.upper())
        self.gesture_var.set(text)
        self.conf_var.set(f"confidence {conf}%")
        self.conf_bar["value"] = max(0, min(100, conf))
        self.gesture_label.configure(fg=DANGER if label == "neutral" else ACCENT)
        self._log(f"gesture {label} {conf}%")

    # ------------------------------------------------------------ capture
    def choose_dir(self) -> None:
        selected = filedialog.askdirectory(initialdir=self.dir_var.get() or ".")
        if selected:
            self.dir_var.set(selected)

    def open_capture(self) -> None:
        if self.capture_file is not None:
            self.close_capture()
        label = self.label_var.get().strip() or "unlabeled"
        safe = re.sub(r"[^A-Za-z0-9_.-]+", "_", label)
        out_dir = Path(self.dir_var.get().strip() or DEFAULT_CAPTURE_DIR)
        try:
            out_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            messagebox.showerror("Capture", f"cannot create {out_dir}: {exc}")
            return
        path = out_dir / f"f407_{safe}_{datetime.now():%Y%m%d_%H%M%S_%f}.csv"
        try:
            self.capture_file = path.open("w", encoding="utf-8", newline="")
        except OSError as exc:
            messagebox.showerror("Capture", f"cannot open {path}: {exc}")
            return
        self.capture_file.write("timestamp,ax,ay,az,gx,gy,gz,label\n")
        self.capture_file.flush()
        self.capture_rows = 0
        self.capture_label = safe
        self.capture_path = path
        self.file_var.set(str(path))
        self._log(f"capture file {path}")

    def write_row(self, line: str) -> None:
        if self.capture_file is None:
            return
        self.capture_file.write(f"{line},{self.capture_label}\n")
        self.capture_rows += 1
        if self.capture_rows % 20 == 0:
            self.capture_file.flush()
            self.file_var.set(f"{self.capture_path}  ({self.capture_rows} rows)")

    def close_capture(self) -> None:
        if self.capture_file is None:
            return
        try:
            self.capture_file.flush()
            self.capture_file.close()
        except OSError:
            pass
        rows = self.capture_rows
        path = self.capture_path
        self.capture_file = None
        self.capture_path = None
        self.capture_rows = 0
        self.file_var.set(f"saved {path}  ({rows} rows)")
        self._log(f"capture closed: {path} ({rows} rows)")

    # --------------------------------------------------------------- misc
    def _log(self, text: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        self.log.configure(state="normal")
        self.log.insert("end", f"[{stamp}] {text}\n")
        # Keep the log bounded.
        lines = int(self.log.index("end-1c").split(".")[0])
        if lines > 500:
            self.log.delete("1.0", f"{lines - 500}.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def clear_log(self) -> None:
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def _demo_tick(self) -> None:
        labels = ["lift", "move_left", "move_right", "neutral", "tilt_left", "tilt_right"]
        label = labels[self._demo_index % len(labels)]
        self._demo_index += 1
        conf = 96 if label != "neutral" else 100
        self._handle_line(f"gesture,{label},{conf}")
        self._handle_line(f"infer_us,{108000 + (self._demo_index * 137) % 5000}")
        self._samples += 10
        self.sample_var.set(f"samples {self._samples}")
        self.root.after(1200, self._demo_tick)

    def on_close(self) -> None:
        self.close_capture()
        if self.worker is not None:
            self.worker.stop()
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="", help="serial port, e.g. COM6")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--demo", action="store_true", help="show synthetic data, no serial")
    args = parser.parse_args()

    root = tk.Tk()
    app = GestureApp(root, demo=args.demo)
    if args.port:
        app.port_var.set(args.port)
    if args.baud:
        app.baud_var.set(str(args.baud))
    if args.port and not args.demo:
        # Convenience: --port COM6 should connect without an extra click.
        root.after(300, app.connect)
    root.mainloop()


if __name__ == "__main__":
    main()
