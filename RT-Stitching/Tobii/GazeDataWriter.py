"""
GazeDataWriter.py
=================
Tobii 5 眼动仪注视数据采集 + 二进制文件写入器
写入格式供 C++ GazeDataReader 实时读取

共享文件格式 (gaze_data.bin):
  [Header]  16 bytes
    - magic:       uint32  = 0x47415A45 ("GAZE")
    - version:     uint32  = 1
    - count:       uint32  = 当前有效注视点数量 (0 ~ MAX_POINTS)
    - write_seq:   uint32  = 写入序列号 (每次写入递增, C++ 用于检测新数据)

  [Data]    count x 24 bytes  (每个注视点)
    - timestamp_ms: int64   = Unix 毫秒时间戳
    - x:            float64 = 归一化屏幕坐标 [0,1]
    - y:            float64 = 归一化屏幕坐标 [0,1]

使用方法:
    python GazeDataWriter.py
"""

import struct
import time
import os
import threading
import socket
import argparse
from collections import deque


# ============================================================
# TCP 发送端：把 gaze blob 发到板子(RK3588)上的 C++ GazeDataReader
#   带断线重连；连接失败/断开都不影响采集主循环。
# ============================================================
class GazeNetSender:
    def __init__(self, host, port, retry_interval=2.0):
        self.host = host
        self.port = int(port)
        self.retry_interval = retry_interval
        self.sock = None
        self._last_try = 0.0

    def _ensure(self):
        if self.sock is not None:
            return
        now = time.time()
        if now - self._last_try < self.retry_interval:
            return  # 限流：避免板子没开时每帧都阻塞重连
        self._last_try = now
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(1.0)
            s.connect((self.host, self.port))
            s.settimeout(None)
            self.sock = s
            print(f"[GazeNetSender] connected to {self.host}:{self.port}")
        except OSError as e:
            self.sock = None
            # 不再静默：连不上时打印原因，避免“白等不知为何”
            print(f"[GazeNetSender] connect {self.host}:{self.port} FAILED: {e} "
                  f"(every {self.retry_interval}s retry; 本机测试请用 127.0.0.1)")

    def send(self, blob):
        self._ensure()
        if self.sock is None:
            return
        try:
            self.sock.sendall(blob)
        except OSError:
            print("[GazeNetSender] send failed, will reconnect")
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

# ============ 配置参数 ============
# 路径相对【本脚本所在目录】解析，跨平台/换机器都不用改。
# 本脚本在 <工程根>/Tobii/ 下，眼动文件放在 <工程根>/out/build/x64-Release/，
# 与 C++ 端 config.yaml 的 gaze_data_path 指向同一文件。
_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_GAZE_DIR = os.path.join(_PROJECT_ROOT, "out", "build", "x64-Release")
os.makedirs(_GAZE_DIR, exist_ok=True)
GAZE_FILE_PATH = os.path.join(_GAZE_DIR, "gaze_data.bin")
TXT_FILE_PATH = os.path.join(_GAZE_DIR, "gaze_data.txt")

MAX_POINTS = 100                        # 最大保留注视点数 (滑动窗口)
GAZE_WINDOW_SEC = 3.0                   # 注视数据时间窗口 (秒)
INTERVAL = 1.0 / 33                     # 采集间隔 ~30ms (匹配 Tobii 5 的 33Hz)
RECORDING_DURATION = 0                  # 记录时长 (秒), 0=无限
WRITE_TXT_ENABLED = True                # 同时写入 txt 日志 (调试用)

# ============ 文件格式常量 ============
MAGIC = 0x47415A45       # "GAZE" ascii
VERSION = 1
HEADER_FORMAT = '<IIII'  # magic, version, count, write_seq (little-endian)
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)  # 16 bytes
POINT_FORMAT = '<qdd'    # timestamp_ms(int64), x(float64), y(float64)
POINT_SIZE = struct.calcsize(POINT_FORMAT)    # 24 bytes

class GazeDataWriter:
    """
    从 Tobii 眼动仪采集注视数据, 写入二进制共享文件供 C++ 端实时读取。
    """

    def __init__(self, file_path=GAZE_FILE_PATH, max_points=MAX_POINTS,
                 window_sec=GAZE_WINDOW_SEC, net_sender=None, write_file=True):
        self.file_path = file_path
        self.max_points = max_points
        self.window_sec = window_sec
        self.write_seq = 0
        self.gaze_buffer = deque(maxlen=max_points)
        self.lock = threading.Lock()
        self.net_sender = net_sender   # GazeNetSender 或 None
        self.write_file = write_file   # 是否仍写本地文件(Windows 本机测试用)

        # 初始化眼动仪
        self._tracker_ok = False
        try:
            import TobiiEyeTracker
            try:
                TobiiEyeTracker.init()
            except:
                pass  # init 返回值有 bug，忽略即可
            self._tracker_ok = True
            print("[GazeDataWriter] Tobii eye tracker initialized")
        except ImportError as e:
            print(f"[GazeDataWriter] TobiiEyeTracker.pyd not found: {e}, using dummy data")

        # 创建初始空文件
        self._write_file()

    def poll_and_write(self):
        """
        单次采集 + 写入循环体。
        """
        points = self._fetch_gaze_points()

        with self.lock:
            now_ms = int(time.time() * 1000)
            self.gaze_buffer.extend(points)
            # 淘汰超过时间窗口的数据
            cutoff_ms = now_ms - int(self.window_sec * 1000)
            while self.gaze_buffer and self.gaze_buffer[0][0] < cutoff_ms:
                self.gaze_buffer.popleft()
            self._write_file()

        # 控制台打印注视点信息
        if points:
            last_x, last_y = points[-1][1], points[-1][2]
            print(f"Current location: ({last_x}, {last_y})")
            print(f"Data written to {self.file_path}")
            print(f"Reading:{len(points)}")

        if WRITE_TXT_ENABLED and points:
            self._write_txt(points)

    def _fetch_gaze_points(self):
        """
        从 Tobii SDK 获取注视点, 返回 [(timestamp_ms, x, y), ...]
        x, y 为归一化屏幕坐标 [0, 1]
        """
        if not self._tracker_ok:
            # 无眼动仪时生成模拟数据 (调试用)
            import random
            now_ms = int(time.time() * 1000)
            x = 0.5 + random.gauss(0, 0.05)
            y = 0.5 + random.gauss(0, 0.05)
            return [(now_ms, max(0.0, min(1.0, x)), max(0.0, min(1.0, y)))]

        try:
            import TobiiEyeTracker
            raw = TobiiEyeTracker.getBuffer()  # 返回 ((x,y), (x,y), ...)
            now_ms = int(time.time() * 1000)
            result = []
            n = len(raw)
            for i, (rx, ry) in enumerate(raw):
                # 为缓冲区内各点分配时间戳
                dt = int(INTERVAL * 1000) if n <= 1 else int(INTERVAL * 1000 * i / n)
                ts = now_ms - int(INTERVAL * 1000) + dt
                x = max(0.0, min(1.0, float(rx)))
                # ★ 修复：不翻转 Y 轴，Tobii 坐标系与图像坐标系一致
                #   (0,0) = 左上角, (1,1) = 右下角
                y = max(0.0, min(1.0, float(ry)))
                result.append((ts, x, y))
            return result
        except Exception as e:
            print(f"[GazeDataWriter] getBuffer error: {e}")
            return []

    def _write_file(self):
        """
        原子写入二进制文件。使用 tmp + rename 防止 C++ 端读到半写数据。
        """
        self.write_seq += 1
        count = len(self.gaze_buffer)

        header = struct.pack(HEADER_FORMAT, MAGIC, VERSION, count, self.write_seq)
        data_parts = [struct.pack(POINT_FORMAT, ts, x, y)
                      for (ts, x, y) in self.gaze_buffer]
        blob = header + b"".join(data_parts)   # 与文件内容完全一致的字节流

        # (a) socket 发送：与 C++ GazeDataReader 的自描述帧一致(先 header 再 points)
        if self.net_sender is not None:
            self.net_sender.send(blob)

        # (b) 写本地文件(可关)：Windows 本机用文件模式测试时仍需要
        if not self.write_file:
            return

        tmp_path = self.file_path + ".tmp"
        with open(tmp_path, 'wb') as f:
            f.write(blob)

        # 原子替换
        try:
            if os.path.exists(self.file_path):
                os.remove(self.file_path)
            os.rename(tmp_path, self.file_path)
        except OSError:
            # Windows 下文件被占用时直接覆写
            with open(self.file_path, 'wb') as f:
                f.write(blob)

    def _write_txt(self, points):
        """追加写入 txt 日志 (兼容原 Send_and_record.py 格式)"""
        try:
            with open(TXT_FILE_PATH, 'a') as f:
                for ts, x, y in points:
                    f.write(f"{ts}, {x:.6f}, {y:.6f}\n")
        except Exception as e:
            print(f"[GazeDataWriter] txt write error: {e}")

    def run_loop(self, duration=0):
        """主循环: duration=0 表示无限运行"""
        start = time.time()
        print(f"[GazeDataWriter] Writing to {self.file_path} (max {self.max_points} pts, window {self.window_sec}s)")
        try:
            while True:
                if duration > 0 and (time.time() - start) >= duration:
                    print("[GazeDataWriter] Duration reached, stopping")
                    break
                self.poll_and_write()
                time.sleep(INTERVAL)
        except KeyboardInterrupt:
            print("[GazeDataWriter] Stopped by user")

# ============================================================
# 独立运行
# ============================================================
if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Tobii 注视数据写入/发送器")
    ap.add_argument("--net", default=None,
                    help="把注视数据发到板子: 形如 192.168.1.50:5599 "
                         "(host:port)。本机测试可用 127.0.0.1:5599")
    ap.add_argument("--no_file", action="store_true",
                    help="不再写本地 gaze_data.bin(纯 socket 模式)")
    args = ap.parse_args()

    sender = None
    if args.net:
        host, _, port = args.net.partition(":")
        sender = GazeNetSender(host, port or "5599")
        print(f"[GazeDataWriter] socket 发送已启用 -> {host}:{port or '5599'}")

    writer = GazeDataWriter(net_sender=sender,
                            write_file=(not args.no_file))
    try:
        writer.run_loop(duration=RECORDING_DURATION)
    finally:
        if sender:
            sender.close()

# ============================================================
# 集成到已有 Send_and_record.py 的示例:
# ============================================================
#
# 在 WebSocketGazeTracker.__init__ 中添加:
#     from GazeDataWriter import GazeDataWriter
#     self.gaze_writer = GazeDataWriter()
#
# 在 get_current_location 方法末尾添加:
#     self.gaze_writer.poll_and_write()
#
# 这样既保留 WebSocket 功能, 又同时产生 gaze_data.bin 供 C++ 端读取。
