#!/usr/bin/env python3
import csv
import datetime as dt
import json
import socket
import struct
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk, messagebox

try:
    from PIL import Image, ImageTk
except Exception:
    Image = None
    ImageTk = None


ROOT = Path(__file__).resolve().parent
CONFIG_PATH = ROOT / "pid_gui_params.json"
DATASET_ROOT = ROOT / "logs" / "datasets"


PARAM_SPECS = [
    ("base_target_speed", 0, 1500, 1),
    ("min_target_speed", 0, 1500, 1),
    ("turn_speed_max", 0, 3000, 1),
    ("target_speed_max", 0, 3000, 1),
    ("lookahead_row", 0, 59, 0.1),
    ("turn_kp", 0, 80, 0.1),
    ("turn_kd", 0, 80, 0.1),
    ("speed_kp_left", 0, 80, 0.1),
    ("speed_ki_left", 0, 20, 0.01),
    ("speed_kd_left", 0, 80, 0.1),
    ("speed_kp_right", 0, 80, 0.1),
    ("speed_ki_right", 0, 20, 0.01),
    ("speed_kd_right", 0, 80, 0.1),
    ("pwm_min", -6000, 0, 0.1),
    ("pwm_max", 0, 6000, 10),
    ("min_start_pwm", 0, 6000, 10),
]

PARAM_LABELS = {
    "turn_kp": "Turn Kp",
    "turn_kd": "Turn Kd",
    "lookahead_row": "Lookahead row",
    "base_target_speed": "Base speed",
    "min_target_speed": "Min speed",
    "turn_speed_max": "Turn max",
    "target_speed_max": "Target max",
    "speed_kp_left": "Left Kp",
    "speed_ki_left": "Left Ki",
    "speed_kd_left": "Left Kd",
    "speed_kp_right": "Right Kp",
    "speed_ki_right": "Right Ki",
    "speed_kd_right": "Right Kd",
    "pwm_min": "PWM min",
    "pwm_max": "PWM max",
    "min_start_pwm": "Start PWM",
}

PARAM_GROUPS = [
    ("Steering", ["turn_kp", "turn_kd", "lookahead_row"]),
    ("Speed", ["base_target_speed", "min_target_speed", "turn_speed_max", "target_speed_max"]),
    ("Speed PID", ["speed_kp_left", "speed_ki_left", "speed_kd_left", "speed_kp_right", "speed_ki_right", "speed_kd_right"]),
    ("PWM", ["pwm_min", "pwm_max", "min_start_pwm"]),
]


def load_config():
    if CONFIG_PATH.exists():
        with CONFIG_PATH.open("r", encoding="utf-8") as f:
            return json.load(f)
    return {
        "board_ip": "192.168.31.187",
        "control_port": 9090,
        "image_port": 8080,
        "status_port": 8083,
        "params": {},
    }


def save_config(cfg):
    with CONFIG_PATH.open("w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2, ensure_ascii=False)
        f.write("\n")


class PidGui:
    def __init__(self, root):
        self.root = root
        self.root.title("Loongson 7.8 Binary Track Telemetry")
        self.cfg = load_config()
        self.board_ip = tk.StringVar(value=self.cfg.get("board_ip", "192.168.31.187"))
        self.control_port = int(self.cfg.get("control_port", 9090))
        self.image_port = int(self.cfg.get("image_port", 8080))
        self.status_port = int(self.cfg.get("status_port", 8083))

        self.control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_sock.settimeout(0.2)
        self.running = True
        self.latest_image = None
        self.latest_status = {}
        self.param_vars = {}
        self.value_labels = {}
        self.dataset_lock = threading.Lock()
        self.dataset_dir = self._create_dataset_dir()
        self.frames_dir = self.dataset_dir / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.latest_frame_path = self.dataset_dir / "latest_frame.jpg"
        self.status_log_path = self.dataset_dir / "status.log"
        self.csv_path = self.dataset_dir / "samples.csv"
        self.csv_file = self.csv_path.open("a", newline="", encoding="utf-8")
        self.csv_writer = None
        self.csv_fields = []
        self.frame_paths = {}
        self.sample_index = 0

        self._build_ui()
        self._start_receivers()
        self.root.after(100, self._refresh_ui)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=8)
        top.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        left = ttk.Frame(top)
        left.grid(row=0, column=0, sticky="nsew")
        right = ttk.Frame(top)
        right.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
        top.columnconfigure(0, weight=1)
        top.columnconfigure(1, weight=0)
        top.rowconfigure(0, weight=1)

        self.image_label = ttk.Label(left, text="waiting for binary track image on UDP 8080", anchor="center")
        self.image_label.grid(row=0, column=0, sticky="nsew")
        self.image_label.configure(background="#202020")
        left.columnconfigure(0, weight=1)
        left.rowconfigure(0, weight=1)

        conn = ttk.Frame(right)
        conn.grid(row=0, column=0, sticky="ew")
        ttk.Label(conn, text="Board IP").grid(row=0, column=0, sticky="w")
        ttk.Entry(conn, textvariable=self.board_ip, width=16).grid(row=0, column=1, padx=4)
        ttk.Button(conn, text="Show", command=lambda: self.send_command("show")).grid(row=0, column=2, padx=2)
        ttk.Button(conn, text="Start", command=lambda: self.send_command("start")).grid(row=0, column=3, padx=2)
        ttk.Button(conn, text="Stop", command=lambda: self.send_command("stop")).grid(row=0, column=4, padx=2)

        param_frame = ttk.Frame(right)
        param_frame.grid(row=1, column=0, sticky="nsew", pady=8)
        right.rowconfigure(1, weight=1)
        right.columnconfigure(0, weight=1)

        defaults = self.cfg.get("params", {})
        for name, low, _high, _step in PARAM_SPECS:
            var = tk.DoubleVar(value=float(defaults.get(name, low)))
            self.param_vars[name] = var

        for group_row, (title, names) in enumerate(PARAM_GROUPS):
            group = ttk.LabelFrame(param_frame, text=title, padding=6)
            group.grid(row=group_row, column=0, sticky="w", pady=(0, 6))
            for row, name in enumerate(names):
                self.add_param_row(group, row, name)
        param_frame.columnconfigure(0, weight=1)

        actions = ttk.Frame(right)
        actions.grid(row=2, column=0, sticky="ew")
        ttk.Button(actions, text="Apply All", command=self.apply_all).grid(row=0, column=0, padx=2)
        ttk.Button(actions, text="Save Local", command=self.save_local).grid(row=0, column=1, padx=2)
        ttk.Button(actions, text="Board Save", command=lambda: self.send_command("save")).grid(row=0, column=2, padx=2)
        ttk.Button(actions, text="Open Dataset", command=self.open_dataset).grid(row=0, column=3, padx=2)

        self.status_text = tk.Text(right, height=12, width=54)
        self.status_text.grid(row=3, column=0, sticky="nsew", pady=(8, 0))
        right.rowconfigure(3, weight=1)

        self.dataset_label = ttk.Label(right, text=f"Dataset: {self.dataset_dir}", anchor="w")
        self.dataset_label.grid(row=4, column=0, sticky="ew", pady=(4, 0))

    def _create_dataset_dir(self):
        now = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        dataset_dir = DATASET_ROOT / f"run_{now}"
        dataset_dir.mkdir(parents=True, exist_ok=True)
        return dataset_dir

    def add_param_row(self, parent, row, name):
        spec = next(s for s in PARAM_SPECS if s[0] == name)
        _name, _low, _high, step = spec
        ttk.Label(parent, text=PARAM_LABELS.get(name, name), width=13).grid(row=row, column=0, sticky="w")
        ttk.Button(parent, text="-", width=3, command=lambda n=name: self.step_param(n, -0.1)).grid(row=row, column=1, padx=2, pady=1)
        entry = ttk.Entry(parent, width=9)
        entry.grid(row=row, column=2, padx=2, pady=1)
        entry.insert(0, self._format_value(self.param_vars[name].get(), step))
        entry.bind("<Return>", lambda _e, n=name, ent=entry: self._entry_apply(n, ent))
        entry.bind("<FocusOut>", lambda _e, n=name, ent=entry: self._entry_apply(n, ent))
        ttk.Button(parent, text="+", width=3, command=lambda n=name: self.step_param(n, 0.1)).grid(row=row, column=3, padx=2, pady=1)
        ttk.Button(parent, text="Set", width=5, command=lambda n=name: self.send_param(n)).grid(row=row, column=4, padx=2, pady=1)
        self.value_labels[name] = entry

    def _start_receivers(self):
        threading.Thread(target=self._image_loop, daemon=True).start()
        threading.Thread(target=self._status_loop, daemon=True).start()

    def _bind_udp(self, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", port))
        sock.settimeout(0.2)
        return sock

    def _image_loop(self):
        sock = self._bind_udp(self.image_port)
        while self.running:
            try:
                data, _addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) < 12 or data[:4] != b"LQIM":
                continue
            seq, jpeg_len = struct.unpack_from("<II", data, 4)
            jpeg = data[12:12 + jpeg_len]
            self._save_frame(seq, jpeg)
            if Image is None:
                continue
            try:
                import io
                image = Image.open(io.BytesIO(jpeg)).convert("RGB")
                self.latest_image = image
            except Exception:
                continue

    def _status_loop(self):
        sock = self._bind_udp(self.status_port)
        while self.running:
            try:
                data, _addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            text = data.decode("utf-8", errors="replace")
            status = {}
            for line in text.splitlines():
                if "=" in line:
                    k, v = line.split("=", 1)
                    status[k.strip()] = v.strip()
            status["_raw"] = text
            self.latest_status = status
            self._save_status(status)

    def _timestamp(self):
        return dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]

    def _save_frame(self, seq, jpeg):
        if not jpeg:
            return
        ts = self._timestamp()
        frame_name = f"img{int(seq):06d}_{ts}.jpg"
        frame_path = self.frames_dir / frame_name
        try:
            frame_path.write_bytes(jpeg)
            self.latest_frame_path.write_bytes(jpeg)
        except OSError:
            return
        with self.dataset_lock:
            self.frame_paths[int(seq)] = frame_path
            if len(self.frame_paths) > 3000:
                for old_seq in sorted(self.frame_paths)[:-2000]:
                    self.frame_paths.pop(old_seq, None)

    def _save_status(self, status):
        raw = status.get("_raw", "")
        ts = self._timestamp()
        try:
            with self.status_log_path.open("a", encoding="utf-8") as f:
                f.write(f"--- {ts} ---\n")
                f.write(raw)
                if not raw.endswith("\n"):
                    f.write("\n")
        except OSError:
            pass

        seq_text = status.get("status_image_seq", "")
        try:
            seq = int(seq_text)
        except ValueError:
            seq = -1

        with self.dataset_lock:
            frame_path = self.frame_paths.get(seq)
        if frame_path is None and seq >= 0:
            time.sleep(0.03)
            with self.dataset_lock:
                frame_path = self.frame_paths.get(seq)

        with self.dataset_lock:
            self.sample_index += 1
            row = {
                "sample_index": self.sample_index,
                "timestamp": ts,
                "status_image_seq": seq_text,
                "frame_path": str(frame_path.relative_to(self.dataset_dir)) if frame_path else "",
                "terminal_line": self._terminal_line(status),
                "target_diff": self._status_delta(status, "target_l", "target_r"),
            }
            for key, value in status.items():
                if key != "_raw":
                    row[key] = value
            self._write_csv_row(row)

    def _terminal_line(self, status):
        if status.get("terminal_line"):
            return status["terminal_line"]
        try:
            return "err={err:4d}  pidL={pid_l:6d}  pidR={pid_r:6d}  spdL={spd_l:4d}  spdR={spd_r:4d}".format(
                err=int(status.get("err", 0)),
                pid_l=int(status.get("pid_l", 0)),
                pid_r=int(status.get("pid_r", 0)),
                spd_l=int(status.get("spd_l", 0)),
                spd_r=int(status.get("spd_r", 0)),
            )
        except ValueError:
            return ""

    def _status_delta(self, status, left_key, right_key):
        try:
            return int(status.get(left_key, 0)) - int(status.get(right_key, 0))
        except ValueError:
            return ""

    def _write_csv_row(self, row):
        keys = list(row.keys())
        if self.csv_writer is None:
            self.csv_fields = keys
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.csv_fields)
            if self.csv_path.stat().st_size == 0:
                self.csv_writer.writeheader()
        else:
            new_keys = [key for key in keys if key not in self.csv_fields]
            if new_keys:
                old_rows = []
                self.csv_file.flush()
                with self.csv_path.open("r", newline="", encoding="utf-8") as f:
                    reader = csv.DictReader(f)
                    old_rows = list(reader)
                self.csv_fields.extend(new_keys)
                self.csv_file.close()
                with self.csv_path.open("w", newline="", encoding="utf-8") as f:
                    writer = csv.DictWriter(f, fieldnames=self.csv_fields)
                    writer.writeheader()
                    writer.writerows(old_rows)
                self.csv_file = self.csv_path.open("a", newline="", encoding="utf-8")
                self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.csv_fields)
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def _refresh_ui(self):
        if self.latest_image is not None and ImageTk is not None:
            w = max(640, self.image_label.winfo_width())
            h = max(480, self.image_label.winfo_height())
            src_w, src_h = self.latest_image.size
            scale = max(4, min(w // max(1, src_w), h // max(1, src_h)))
            resample = getattr(getattr(Image, "Resampling", Image), "NEAREST")
            image = self.latest_image.resize((src_w * scale, src_h * scale), resample)
            photo = ImageTk.PhotoImage(image)
            self.image_label.configure(image=photo, text="")
            self.image_label.image = photo
        elif Image is None:
            self.image_label.configure(text="Install Pillow to display JPEG: pip install pillow")

        if self.latest_status:
            self.status_text.delete("1.0", "end")
            self.status_text.insert("1.0", self.latest_status.get("_raw", ""))
        self.root.after(100, self._refresh_ui)

    def _format_value(self, value, step):
        return f"{value:.3f}".rstrip("0").rstrip(".")

    def update_param_entry(self, name):
        spec = next(s for s in PARAM_SPECS if s[0] == name)
        _name, _low, _high, step = spec
        value = self.param_vars[name].get()
        entry = self.value_labels[name]
        entry.delete(0, "end")
        entry.insert(0, self._format_value(value, step))

    def step_param(self, name, delta):
        spec = next(s for s in PARAM_SPECS if s[0] == name)
        _name, low, high, _step = spec
        value = max(low, min(high, self.param_vars[name].get() + delta))
        value = round(value, 3)
        self.param_vars[name].set(value)
        self.update_param_entry(name)
        self.send_param(name)

    def _entry_apply(self, name, entry):
        try:
            value = float(entry.get())
        except ValueError:
            return
        self.param_vars[name].set(value)
        self.send_param(name)

    def target(self):
        return (self.board_ip.get().strip(), self.control_port)

    def send_command(self, cmd):
        try:
            self.control_sock.sendto(cmd.encode("utf-8"), self.target())
        except OSError as exc:
            messagebox.showerror("UDP send failed", str(exc))

    def send_param(self, name):
        value = self.param_vars[name].get()
        self.send_command(f"{name}={value:g}")

    def apply_all(self):
        for name, _low, _high, _step in PARAM_SPECS:
            self.send_param(name)
            time.sleep(0.01)

    def save_local(self):
        self.cfg["board_ip"] = self.board_ip.get().strip()
        self.cfg["control_port"] = self.control_port
        self.cfg["image_port"] = self.image_port
        self.cfg["status_port"] = self.status_port
        self.cfg["params"] = {name: self.param_vars[name].get() for name, *_ in PARAM_SPECS}
        save_config(self.cfg)

    def open_dataset(self):
        try:
            import os
            os.startfile(str(self.dataset_dir))
        except Exception as exc:
            messagebox.showerror("Open dataset failed", str(exc))

    def close(self):
        self.save_local()
        self.running = False
        try:
            self.csv_file.close()
        except Exception:
            pass
        self.root.destroy()


def main():
    root = tk.Tk()
    root.geometry("1380x860")
    PidGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
