import websockets
import time
import asyncio
import json
from collections import deque
import TobiiEyeTracker

# 文件路径
OUTPUT_FILE = "gaze_data.txt"

class WebSocketGazeTracker:
    def __init__(self):
        self.clients = set()
        self.gaze_queue = deque(maxlen=CACHE_SIZE)
        self.last_location = None
        self.start_time = None  # 记录程序启动时间
        self.recording = False  # 记录是否正在记录数据
        self.write_to_file_enabled = WRITE_TO_FILE_ENABLED  # 控制是否写入文件
        try:
            TobiiEyeTracker.init()
        except:
            pass
        print("Eye tracker initialized")

    async def handler(self, websocket, path):
        """
        WebSocket 连接处理函数
        """
        self.clients.add(websocket)
        try:
            async for message in websocket:
                print(f"Received message: {message}")
        except websockets.ConnectionClosed:
            print("Connection closed")
        finally:
            self.clients.remove(websocket)

    async def update_gaze_data(self):
        if RECORDING_DURATION_ENABLED:  # 如果启用了记录时长功能
            self.start_time = time.time()  # 记录程序启动时间
            self.recording = True
            print(f"Recording started. Will stop after {RECORDING_DURATION} seconds.")

        while True:
            if RECORDING_DURATION_ENABLED and self.recording:
                elapsed_time = time.time() - self.start_time
                if elapsed_time >= RECORDING_DURATION:
                    print("Recording duration reached. Stopping...")
                    self.recording = False
                    break  # 退出循环，停止数据更新

            self.get_current_location()
            await self.progressComm()
            await asyncio.sleep(INTERVAL)

    async def progressComm(self):
        if self.last_location and self.clients:
            message = json.dumps({"type": "location", "data": self.last_location})
            await asyncio.wait([client.send(message) for client in self.clients])
            print(f"Sent location: {self.last_location}")
        else:
            print("No clients connected. Skipping message send.")

    def get_current_location(self):
        try:
            gaze_data = TobiiEyeTracker.getBuffer()
            gaze_data_time = [(x, 1 - y, int(time.time() * 1000)) for x, y in gaze_data]  # 转换为时间戳

            if gaze_data:
                self.last_location = (gaze_data[-1][0], 1 - gaze_data[-1][1])  # 获取最近的点并转换 y 坐标
                print(f"Current location: {self.last_location}")
                self.gaze_queue.extend(gaze_data_time)  # 将所有新数据加入队列
            else:
                self.last_location = (0, 0)  # 若没有数据，则置为 (0, 0)
                print(f"Current location: {self.last_location}")
                gaze_data_time = [(0, 0, int(time.time() * 1000))]  # 构造一个默认数据点

            # 如果启用了写入文件功能，则将数据写入文件
            if self.write_to_file_enabled:
                self.write_to_file(gaze_data_time)
        except Exception as e:
            print(f"Error in get_current_location: {e}")

    def write_to_file(self, data):
        try:
            with open(OUTPUT_FILE, "a") as f:  # 以追加模式打开文件
                for x, y, timestamp in data:
                    f.write(f"{timestamp}, {x}, {y}\n")  # 写入时间戳、x 坐标、y 坐标
            print(f"Data written to {OUTPUT_FILE}")
        except Exception as e:
            print(f"Error writing to file: {e}")

    async def run(self):
        async with websockets.serve(self.handler, "localhost", 6789):
            print("Server started on ws://localhost:6789")
            await asyncio.gather(
                self.update_gaze_data(),  # 启动数据更新任务
                return_exceptions=True  # 忽略任务中的异常
            )

    def start(self):
        asyncio.run(self.run())  # 使用 asyncio.run() 启动事件循环


if __name__ == "__main__":
    INTERVAL = 1/33  # 更新间隔（秒）
    CACHE_SIZE = 3000  # 缓存长度（秒）
    RECORDING_DURATION = 120  # 记录时长（秒）
    RECORDING_DURATION_ENABLED = True  # 是否启用记录时长功能
    WRITE_TO_FILE_ENABLED = True  # 是否启用文件写入功能

    tracker = WebSocketGazeTracker()    # 创建 WebSocket 服务对象
    tracker.start()  # 启动服务