#!/usr/bin/env python3
"""
任务02 闭环标定脚本

用途：新 MPU6050 到货后，一条命令跑完全部指标并输出表格。
用法：
    python docs/closed_loop_test.py            # 默认 COM3
    python docs/closed_loop_test.py COM5       # 指定串口

它会做：
  1. 停机 + 中位 + 校准
  2. 静态抖动（3 秒）
  3. 阶跃 +6° / -6° / +12°（各 4 秒）
  4. 重复指向精度（连续 5 次回 0）
  5. 打印汇总表，可直接粘进 docs/16
"""
import statistics
import sys
import time

try:
    import serial
except ImportError:
    print("需要 pyserial： pip install pyserial")
    sys.exit(1)

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"
BAUD = 115200


class Link:
    def __init__(self, port):
        self.sp = serial.Serial(port, BAUD, timeout=0.3)
        self.buf = b""

    def send(self, cmd):
        self.sp.write((cmd + "\r\n").encode())

    def read_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            try:
                n = self.sp.in_waiting
                if n:
                    self.buf += self.sp.read(n)
            except Exception:
                pass
            time.sleep(0.01)
        text, _, self.buf = self.buf.decode("latin1").rpartition("\r\n")
        return text.split("\r\n") if text else []

    def clear(self):
        try:
            self.sp.reset_input_buffer()
        except Exception:
            pass
        self.buf = b""

    def pitches(self, lines):
        out = []
        for ln in lines:
            if ln.startswith("A,"):
                parts = ln.split(",")
                if len(parts) == 4:
                    out.append(int(parts[2]) / 100.0)
        return out

    def snapshot(self, lines):
        for ln in reversed(lines):
            if " servo=" in ln:
                return ln
        return ""


def stats(values):
    if not values:
        return None
    return {
        "n": len(values),
        "avg": statistics.fmean(values),
        "pkpk": max(values) - min(values),
        "std": statistics.pstdev(values) if len(values) > 1 else 0.0,
    }


def measure(link, target_deg, settle_s=3.0, window_s=2.0):
    link.send(f"T,0,{int(round(target_deg * 100))}")
    time.sleep(settle_s)
    link.clear()
    lines = link.read_for(window_s)
    ps = link.pitches(lines)
    return stats(ps)


def main():
    link = Link(PORT)
    print(f"串口 {PORT} 已连接")
    link.clear()

    print("\n[1/5] 停机 + 回中位")
    link.send("D")
    time.sleep(0.4)
    link.send("P,1500,1850")     # 行程中间（按实际机械调整）
    time.sleep(3.5)

    print("[2/5] 上电自校准")
    link.send("C")
    time.sleep(3.5)
    link.clear()
    lines = link.read_for(1.5)
    print("   ", link.snapshot(lines)[-70:])
    zero = stats(link.pitches(lines))
    if zero:
        print(f"    校准后角度 {zero['avg']:+.2f}°（应接近 0）")

    print("[3/5] 静态抖动（目标 0°）")
    link.send("G,100,30,5")      # kp=1.0 kd=0.3 ki=0.05
    time.sleep(0.2)
    link.send("X")
    time.sleep(4.0)
    st = measure(link, 0.0)
    if st:
        print(f"    均值 {st['avg']:+.2f}°  峰峰 {st['pkpk']:.2f}°  标准差 {st['std']:.3f}°")

    print("[4/5] 阶跃")
    steps = []
    for tgt in (6.0, -6.0, 12.0, 0.0):
        r = measure(link, tgt)
        if r:
            err = abs(tgt - r["avg"])
            steps.append((tgt, r["avg"], err, r["pkpk"]))
            print(f"    目标 {tgt:+6.1f}° → 实际 {r['avg']:+6.2f}°  误差 {err:5.2f}°  峰峰 {r['pkpk']:.2f}°")
        time.sleep(0.5)

    print("[5/5] 重复指向精度（回 0° 五次）")
    reps = []
    for _ in range(5):
        measure(link, 10.0, settle_s=2.5, window_s=1.2)
        r = measure(link, 0.0, settle_s=2.5, window_s=1.2)
        if r:
            reps.append(r["avg"])
    if reps:
        span = max(reps) - min(reps)
        print(f"    五次回零：{[round(v,2) for v in reps]}")
        print(f"    重复精度（极差）{span:.2f}°")

    link.send("D")

    print("\n" + "=" * 56)
    print("汇总（可直接粘进 docs/16-闭环实测记录.md）")
    print("=" * 56)
    if st:
        verdict = "✅" if st["pkpk"] <= 2.0 else "❌"
        print(f"| 静态抖动峰峰 | ≤2.0° | {st['pkpk']:.2f}° | {verdict} |")
    for tgt, act, err, pk in steps:
        verdict = "✅" if err <= max(0.05 * abs(tgt), 2.0) else "❌"
        print(f"| 阶跃 {tgt:+.0f}° 稳态误差 | ≤5% | {err:.2f}° | {verdict} |")
    if reps:
        verdict = "✅" if span <= 2.0 else "❌"
        print(f"| 重复指向精度 | ≤±2° | {span:.2f}° | {verdict} |")


if __name__ == "__main__":
    main()
