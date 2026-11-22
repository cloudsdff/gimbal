#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务02 上位机：实时绘制三轴角度波形 + 在线调参

协议（MCU → PC，文本行，115200 8N1）：
    A,<yaw_cd>,<pitch_cd>,<roll_cd>\r\n     角度，单位 0.01°（整数）
    （可选）S,...                          其他状态行，本版本忽略

协议（PC → MCU）：
    K,<alpha_milli>\r\n      设置互补滤波系数，例 K,980 = 0.980
    T,<yaw_cd>,<pitch_cd>\r\n 设置目标指向（后续云台闭环用）
    C\r\n                    重新执行上电自校准

用法：
    python app.py              # 打开界面（默认模拟模式）
    python app.py --selftest   # 无界面自检，验证解析与绘图逻辑
"""

import math
import queue
import random
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk

try:
    import serial  # pyserial
    from serial.tools import list_ports
    HAS_SERIAL = True
except ImportError:  # 没装 pyserial 也能用模拟模式
    HAS_SERIAL = False
    list_ports = None

BAUD_DEFAULT = 115200
WINDOW_SAMPLES = 500          # 画面上保留的采样点数（50 Hz 下 = 10 秒）
DRAW_INTERVAL_MS = 33         # 约 30 FPS


# ---------------------------------------------------------------- 数据解析
def parse_line(line: str):
    """把一行文本解析成 (yaw, pitch, roll)，单位度；解析失败返回 None"""
    line = line.strip()
    if not line.startswith("A,"):
        return None
    parts = line.split(",")
    if len(parts) != 4:
        return None
    try:
        yaw = int(parts[1]) / 100.0
        pitch = int(parts[2]) / 100.0
        roll = int(parts[3]) / 100.0
    except ValueError:
        return None
    return yaw, pitch, roll


def encode_alpha(alpha: float) -> bytes:
    return ("K,%d\r\n" % int(round(alpha * 1000))).encode("ascii")


def encode_target(yaw_deg: float, pitch_deg: float) -> bytes:
    return ("T,%d,%d\r\n" % (int(yaw_deg * 100), int(pitch_deg * 100))).encode("ascii")


def encode_calibrate() -> bytes:
    return b"C\r\n"


# ---------------------------------------------------------------- 串口读取线程
class SerialReader(threading.Thread):
    def __init__(self, port, baud, out_queue):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.q = out_queue
        self._stop = threading.Event()
        self._ser = None
        self.errors = 0

    def run(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.2)
        except Exception as exc:  # 打不开就在队列里报错
            self.q.put(("error", "打开失败: %s" % exc))
            return
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(256)
            except Exception as exc:
                self.errors += 1
                self.q.put(("error", "读取异常: %s" % exc))
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode("ascii", errors="ignore")
                sample = parse_line(text)
                if sample is not None:
                    self.q.put(("data", sample))
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass

    def write(self, data: bytes):
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(data)
                return True
            except Exception:
                return False
        return False

    def stop(self):
        self._stop.set()


# ---------------------------------------------------------------- 模拟数据源
class Simulator(threading.Thread):
    """没有硬件时产生正弦 + 噪声的角度数据，方便先把界面跑通"""

    def __init__(self, out_queue, hz=50):
        super().__init__(daemon=True)
        self.q = out_queue
        self.dt = 1.0 / hz
        self._stop = threading.Event()
        self.t = 0.0

    def run(self):
        while not self._stop.is_set():
            self.t += self.dt
            yaw = 30.0 * math.sin(self.t * 0.6)
            pitch = 12.0 * math.sin(self.t * 1.1 + 0.7)
            roll = 8.0 * math.sin(self.t * 1.7 + 1.9)
            noise = lambda: random.uniform(-0.15, 0.15)
            self.q.put(("data", (yaw + noise(), pitch + noise(), roll + noise())))
            time.sleep(self.dt)

    def stop(self):
        self._stop.set()


# ---------------------------------------------------------------- 绘图控件
class Scope(ttk.Frame):
    """用 Canvas 画三条曲线（不依赖 matplotlib）"""

    COLORS = {"yaw": "#4aa3ff", "pitch": "#ffb020", "roll": "#4cd964"}

    def __init__(self, master, width=980, height=420):
        super().__init__(master)
        self.canvas = tk.Canvas(self, width=width, height=height, bg="#12141a",
                                highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.w = width
        self.h = height
        self.series = {"yaw": [], "pitch": [], "roll": []}
        self.auto_scale = tk.BooleanVar(value=True)
        self.ymax = 30.0     # 手动量程（度）

    def push(self, yaw, pitch, roll):
        for key, val in (("yaw", yaw), ("pitch", pitch), ("roll", roll)):
            s = self.series[key]
            s.append(val)
            if len(s) > WINDOW_SAMPLES:
                del s[:len(s) - WINDOW_SAMPLES]

    def clear(self):
        for s in self.series.values():
            s.clear()

    def redraw(self):
        c = self.canvas
        c.delete("all")
        pad_l, pad_r, pad_t, pad_b = 56, 16, 16, 28
        w = max(1, self.w - pad_l - pad_r)
        h = max(1, self.h - pad_t - pad_b)

        # 自动量程
        peak = 1.0
        for s in self.series.values():
            for v in s:
                peak = max(peak, abs(v))
        ymax = max(10.0, math.ceil(peak / 10.0) * 10.0) if self.auto_scale.get() else self.ymax

        # 网格 + Y 轴刻度
        for i in range(5):
            y = pad_t + h * i / 4.0
            c.create_line(pad_l, y, pad_l + w, y, fill="#262a33")
            val = ymax - (2.0 * ymax) * i / 4.0
            c.create_text(pad_l - 8, y, text="%d°" % val, fill="#6b7280",
                          anchor="e", font=("Consolas", 9))
        c.create_line(pad_l, pad_t + h / 2, pad_l + w, pad_t + h / 2, fill="#3a4050")

        # 三条曲线
        n = max(1, WINDOW_SAMPLES - 1)
        for key, s in self.series.items():
            if len(s) < 2:
                continue
            pts = []
            for i, v in enumerate(s):
                x = pad_l + w * (i / n)
                y = pad_t + h / 2 - (v / ymax) * (h / 2)
                pts.extend((x, y))
            c.create_line(*pts, fill=self.COLORS[key], width=2)

        # 图例 + 当前值
        x = pad_l + 10
        for key in ("yaw", "pitch", "roll"):
            val = self.series[key][-1] if self.series[key] else 0.0
            c.create_rectangle(x, pad_t + 4, x + 12, pad_t + 14, fill=self.COLORS[key], outline="")
            c.create_text(x + 18, pad_t + 9, text="%s %7.2f°" % (key.upper(), val),
                          fill="#d0d6e0", anchor="w", font=("Consolas", 11))
            x += 190

        # X 轴说明
        c.create_text(pad_l + w, self.h - 10,
                      text="%d 点（50 Hz ≈ %.0f 秒）" % (WINDOW_SAMPLES, WINDOW_SAMPLES / 50.0),
                      fill="#4b5563", anchor="e", font=("Consolas", 9))


# ---------------------------------------------------------------- 主界面
class App:
    def __init__(self, root, simulate=True):
        self.root = root
        root.title("任务02 上位机 · Yaw-Pitch 姿态监视")
        root.configure(bg="#0e1015")
        self.q = queue.Queue()
        self.reader = None
        self.sim = None
        self.frames = 0
        self.bytes_mode = False
        self.last_fps_t = time.time()
        self.last_fps_frames = 0

        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TFrame", background="#0e1015")
        style.configure("TLabel", background="#0e1015", foreground="#c9d1dc")
        style.configure("TLabelframe", background="#0e1015", foreground="#8b95a5")
        style.configure("TLabelframe.Label", background="#0e1015", foreground="#8b95a5")
        style.configure("TButton", padding=4)

        # ---- 顶部：连接控制 ----
        top = ttk.Frame(root, padding=(8, 6))
        top.pack(fill="x")

        ttk.Label(top, text="串口").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=10, state="readonly")
        self.port_box.pack(side="left", padx=(4, 6))
        self.refresh_ports()

        ttk.Button(top, text="刷新", command=self.refresh_ports).pack(side="left", padx=2)

        ttk.Label(top, text="波特率").pack(side="left", padx=(10, 0))
        self.baud_var = tk.StringVar(value=str(BAUD_DEFAULT))
        ttk.Combobox(top, textvariable=self.baud_var, width=8, state="readonly",
                     values=("9600", "38400", "57600", "115200", "230400")).pack(side="left", padx=4)

        self.sim_var = tk.BooleanVar(value=simulate)
        ttk.Checkbutton(top, text="模拟数据", variable=self.sim_var,
                        command=self.restart_source).pack(side="left", padx=(10, 0))

        self.btn_conn = ttk.Button(top, text="连接", command=self.toggle_connect)
        self.btn_conn.pack(side="left", padx=(10, 4))

        self.btn_cal = ttk.Button(top, text="重新校准", command=self.send_calibrate)
        self.btn_cal.pack(side="left", padx=4)

        # ---- 中部：示波器 ----
        mid = ttk.Frame(root, padding=(8, 0))
        mid.pack(fill="both", expand=True)
        self.scope = Scope(mid)
        self.scope.pack(fill="both", expand=True)

        # ---- 下部：调参 ----
        ctrl = ttk.Labelframe(root, text="在线调参", padding=(8, 6))
        ctrl.pack(fill="x", padx=8, pady=6)

        ttk.Label(ctrl, text="滤波系数 alpha").grid(row=0, column=0, sticky="w")
        self.alpha_var = tk.DoubleVar(value=0.980)
        self.alpha_label = ttk.Label(ctrl, text="0.980", width=7)
        alpha_scale = ttk.Scale(ctrl, from_=0.80, to=0.999, orient="horizontal",
                                variable=self.alpha_var, length=320,
                                command=self.on_alpha_change)
        alpha_scale.grid(row=0, column=1, padx=6)
        self.alpha_label.grid(row=0, column=2)

        ttk.Label(ctrl, text="目标 Yaw").grid(row=0, column=3, padx=(24, 0), sticky="w")
        self.tgt_yaw = tk.DoubleVar(value=0.0)
        ttk.Scale(ctrl, from_=-90, to=90, orient="horizontal", variable=self.tgt_yaw,
                  length=180, command=self.on_target_change).grid(row=0, column=4, padx=6)
        ttk.Label(ctrl, text="目标 Pitch").grid(row=0, column=5, padx=(12, 0), sticky="w")
        self.tgt_pitch = tk.DoubleVar(value=0.0)
        ttk.Scale(ctrl, from_=-45, to=45, orient="horizontal", variable=self.tgt_pitch,
                  length=150, command=self.on_target_change).grid(row=0, column=6, padx=6)

        # ---- 状态栏 ----
        self.status = tk.StringVar(value="就绪")
        bar = ttk.Frame(root, padding=(8, 4))
        bar.pack(fill="x")
        ttk.Label(bar, textvariable=self.status).pack(side="left")

        self.restart_source()
        self.root.after(DRAW_INTERVAL_MS, self.tick)

    # ---------- 数据源切换 ----------
    def refresh_ports(self):
        if HAS_SERIAL:
            ports = [p.device for p in list_ports.comports()]
        else:
            ports = []
        self.port_box["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
        elif not ports:
            self.port_box["values"] = ("(无串口)",)

    def restart_source(self):
        if self.reader:
            self.reader.stop()
            self.reader = None
        if self.sim:
            self.sim.stop()
            self.sim = None
        self.scope.clear()
        self.frames = 0
        if self.sim_var.get():
            self.sim = Simulator(self.q)
            self.sim.start()
            self.status.set("模拟数据中（正弦波，不是真实传感器）")
        else:
            self.status.set("请选择串口后点“连接”")

    def toggle_connect(self):
        if self.reader:
            self.reader.stop()
            self.reader = None
            self.status.set("已断开")
            return
        if not HAS_SERIAL:
            self.status.set("未安装 pyserial：pip install pyserial")
            return
        port = self.port_var.get()
        if not port or port.startswith("("):
            self.status.set("没有可用串口")
            return
        self.sim_var.set(False)
        self.restart_source()
        self.reader = SerialReader(port, int(self.baud_var.get()), self.q)
        self.reader.start()
        self.status.set("已连接 %s @ %s" % (port, self.baud_var.get()))

    # ---------- 下行指令 ----------
    def send_calibrate(self):
        if self.reader and self.reader.write(encode_calibrate()):
            self.status.set("已发送：重新校准")
        else:
            self.status.set("未连接，校准指令未发送")

    def on_alpha_change(self, _=None):
        a = self.alpha_var.get()
        self.alpha_label.configure(text="%.3f" % a)
        if self.reader:
            self.reader.write(encode_alpha(a))

    def on_target_change(self, _=None):
        if self.reader:
            self.reader.write(encode_target(self.tgt_yaw.get(), self.tgt_pitch.get()))

    # ---------- 主循环 ----------
    def tick(self):
        got = 0
        while True:
            try:
                kind, payload = self.q.get_nowait()
            except queue.Empty:
                break
            if kind == "data":
                self.scope.push(*payload)
                self.frames += 1
                got += 1
            elif kind == "error":
                self.status.set(payload)

        self.scope.redraw()

        # 每 5 秒自动重扫串口（未连接时），插拔设备不用手动点刷新
        now = time.time()
        if (not self.reader) and (now - getattr(self, "_last_port_scan", 0.0) >= 5.0):
            self._last_port_scan = now
            self.refresh_ports()

        if now - self.last_fps_t >= 1.0:
            fps = (self.frames - self.last_fps_frames) / (now - self.last_fps_t)
            self.last_fps_t = now
            self.last_fps_frames = self.frames
            src = "模拟" if self.sim_var.get() else "串口"
            self.status.set("%s · 有效帧 %d · 刷新率 %.1f Hz" % (src, self.frames, fps))

        self.root.after(DRAW_INTERVAL_MS, self.tick)


# ---------------------------------------------------------------- 自检
def selftest():
    ok = 0
    fail = 0

    def check(name, cond):
        nonlocal ok, fail
        if cond:
            ok += 1
            print("  [ok] %s" % name)
        else:
            fail += 1
            print("  [FAIL] %s" % name)

    print("== 解析测试 ==")
    check("正常帧", parse_line("A,1234,-560,890") == (12.34, -5.6, 8.9))
    check("带回车", parse_line("A,0,0,0\r") == (0.0, 0.0, 0.0))
    check("非角度行返回 None", parse_line("att 50Hz ...") is None)
    check("字段数不对返回 None", parse_line("A,1,2") is None)
    check("非数字返回 None", parse_line("A,x,2,3") is None)

    print("== 编码测试 ==")
    check("alpha 编码", encode_alpha(0.98) == b"K,980\r\n")
    check("目标编码", encode_target(-30.5, 12.25) == b"T,-3050,1225\r\n")
    check("校准编码", encode_calibrate() == b"C\r\n")

    print("== 缓冲与绘图逻辑 ==")
    s = Scope.__new__(Scope)
    s.series = {"yaw": [], "pitch": [], "roll": []}
    for i in range(WINDOW_SAMPLES + 123):
        s.series["yaw"].append(float(i))
        s.series["pitch"].append(0.0)
        s.series["roll"].append(0.0)
        if len(s.series["yaw"]) > WINDOW_SAMPLES:
            del s.series["yaw"][:len(s.series["yaw"]) - WINDOW_SAMPLES]
    check("环形缓冲不超过上限", len(s.series["yaw"]) == WINDOW_SAMPLES)
    check("保留最新数据", s.series["yaw"][-1] == float(WINDOW_SAMPLES + 122))

    print("\n结果: %d 项通过, %d 项失败" % (ok, fail))
    return 0 if fail == 0 else 1


def main():
    if "--selftest" in sys.argv:
        sys.exit(selftest())

    root = tk.Tk()
    root.geometry("1040x680")
    simulate = "--sim" in sys.argv or "--connect" not in sys.argv
    App(root, simulate=simulate)
    root.mainloop()


if __name__ == "__main__":
    main()
