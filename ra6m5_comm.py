import serial
import time
import struct
import crcmod
import threading
import logging
import sys
from datetime import datetime
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QPushButton, QVBoxLayout, QWidget, QTextEdit
from PyQt5.QtCore import QTimer, Qt

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('/var/log/ra6m5_comm.log'),
        logging.StreamHandler()
    ]
)


class RA6M5Communicator:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.crc16 = crcmod.mkCrcFun(0x11021, rev=False, initCrc=0xFFFF)
        self.lock = threading.Lock()
        self.running = False
        self.data_queue = []
        self.worker_thread = None
        self.last_error = None
        self.open()

    def open(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
                rtscts=False,
                dsrdtr=False,
                exclusive=True  # root专属：独占模式访问设备
            )
            logging.info(f"Connected to {self.port} at {self.baudrate} bps")
            self.last_error = None
            return True
        except serial.SerialException as e:
            self.last_error = str(e)
            logging.error(f"Error opening port: {e}")
            return False

    def close(self):
        self.stop_worker()
        if self.ser and self.ser.is_open:
            self.ser.close()
            logging.info("Serial port closed")

    def reconnect(self, attempts=3, delay=0.5):
        for i in range(attempts):
            try:
                self.close()
                time.sleep(delay)
                if self.open():
                    return True
            except Exception as e:
                logging.error(f"Reconnect attempt {i + 1} failed: {e}")
                time.sleep(delay)
        return False

    def send_command(self, cmd_id, payload=b''):
        if not self.ser or not self.ser.is_open:
            if not self.reconnect():
                return False

        with self.lock:
            try:
                # 帧结构: [SOF 0xAA][LEN][CMD][PAYLOAD][CRC16]
                frame = bytearray()
                frame.append(0xAA)  # Start of Frame
                frame.append(len(payload) + 1)  # Length (CMD + PAYLOAD)
                frame.append(cmd_id)
                frame.extend(payload)

                # 计算CRC16 (从LEN开始到PAYLOAD结束)
                crc = self.crc16(bytes(frame[1:]))
                frame.append(crc & 0xFF)
                frame.append((crc >> 8) & 0xFF)

                self.ser.write(frame)
                logging.debug(f"Sent: {frame.hex(' ')}")
                return True
            except serial.SerialException as e:
                self.last_error = str(e)
                logging.error(f"Send error: {e}")
                return False

    def receive_frame(self, timeout=None):
        if timeout is None:
            timeout = self.timeout

        if not self.ser or not self.ser.is_open:
            return None

        with self.lock:
            try:
                state = "WAIT_SOF"
                frame = bytearray()
                payload_len = 0
                start_time = time.time()

                while time.time() - start_time < timeout:
                    if self.ser.in_waiting:
                        byte = self.ser.read(1)[0]

                        if state == "WAIT_SOF" and byte == 0xAA:
                            state = "WAIT_LEN"
                            frame.append(byte)
                        elif state == "WAIT_LEN":
                            frame.append(byte)
                            payload_len = byte - 1  # 减去CMD长度
                            state = "RECEIVE_DATA"
                        elif state == "RECEIVE_DATA":
                            frame.append(byte)
                            # 检查完整帧: SOF + LEN + CMD + PAYLOAD + CRC(2)
                            if len(frame) >= (3 + payload_len + 2):
                                # 验证CRC
                                crc_received = frame[-2] | (frame[-1] << 8)
                                crc_calculated = self.crc16(bytes(frame[1:-2]))

                                if crc_received == crc_calculated:
                                    logging.debug(f"Received: {frame.hex(' ')}")
                                    return frame
                                else:
                                    logging.warning(f"CRC Error! Recv: {crc_received:04X}, Calc: {crc_calculated:04X}")
                                    return None
                    else:
                        time.sleep(0.001)  # root环境可减少休眠时间

                logging.warning("Receive timeout")
                return None
            except serial.SerialException as e:
                self.last_error = str(e)
                logging.error(f"Receive error: {e}")
                return None

    def start_worker(self):
        if self.worker_thread and self.worker_thread.is_alive():
            return

        self.running = True
        self.worker_thread = threading.Thread(target=self._worker)
        self.worker_thread.daemon = True
        self.worker_thread.start()
        logging.info("Worker thread started")

    def stop_worker(self):
        self.running = False
        if self.worker_thread and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=0.5)
            logging.info("Worker thread stopped")

    def _worker(self):
        """后台工作线程，持续读取数据"""
        while self.running:
            try:
                frame = self.receive_frame(timeout=0.1)
                if frame:
                    # 解析帧数据
                    cmd_id = frame[2]
                    payload = frame[3:-2]  # 去掉SOF、LEN、CMD和CRC

                    # 处理温度数据 (命令ID 0x01)
                    if cmd_id == 0x01 and len(payload) >= 2:
                        temp_raw = (payload[0] << 8) | payload[1]
                        temperature = temp_raw / 10.0
                        timestamp = datetime.now().isoformat()
                        self.data_queue.append({
                            "timestamp": timestamp,
                            "type": "temperature",
                            "value": temperature,
                            "unit": "°C"
                        })

                    # 处理湿度数据 (命令ID 0x02)
                    elif cmd_id == 0x02 and len(payload) >= 1:
                        humidity = payload[0]
                        timestamp = datetime.now().isoformat()
                        self.data_queue.append({
                            "timestamp": timestamp,
                            "type": "humidity",
                            "value": humidity,
                            "unit": "%"
                        })

                    # 处理状态数据 (命令ID 0x03)
                    elif cmd_id == 0x03 and len(payload) >= 4:
                        status = {
                            "voltage": struct.unpack('>H', payload[0:2])[0] / 100.0,
                            "current": struct.unpack('>H', payload[2:4])[0] / 1000.0,
                        }
                        timestamp = datetime.now().isoformat()
                        self.data_queue.append({
                            "timestamp": timestamp,
                            "type": "status",
                            "value": status
                        })

                # root环境可增加轮询频率
                time.sleep(0.01)
            except Exception as e:
                logging.error(f"Worker error: {e}")
                time.sleep(0.5)

    def get_queued_data(self):
        """获取并清空数据队列"""
        with self.lock:
            data = self.data_queue.copy()
            self.data_queue.clear()
            return data

    def read_sensor(self, sensor_id, timeout=1.0):
        """读取指定传感器数据"""
        if self.send_command(sensor_id):
            start_time = time.time()
            while time.time() - start_time < timeout:
                frame = self.receive_frame(timeout=0.1)
                if frame and frame[2] == sensor_id:
                    return self.parse_frame(frame)
            return None
        return None

    def parse_frame(self, frame):
        """解析数据帧"""
        cmd_id = frame[2]
        payload = frame[3:-2]

        if cmd_id == 0x01 and len(payload) >= 2:  # 温度
            temp_raw = (payload[0] << 8) | payload[1]
            return temp_raw / 10.0

        elif cmd_id == 0x02 and len(payload) >= 1:  # 湿度
            return payload[0]

        elif cmd_id == 0x03 and len(payload) >= 4:  # 状态
            return {
                "voltage": struct.unpack('>H', payload[0:2])[0] / 100.0,
                "current": struct.unpack('>H', payload[2:4])[0] / 1000.0,
            }

        return None


class RA6M5ControlGUI(QMainWindow):
    def __init__(self, comm):
        super().__init__()
        self.comm = comm
        self.initUI()

        # 启动数据采集线程
        self.comm.start_worker()

        # 设置定时器更新UI
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(500)  # 每500ms更新一次

    def initUI(self):
        self.setWindowTitle('RA6M5 监控控制台')
        self.setGeometry(300, 300, 600, 400)

        # 创建中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # 状态标签
        self.status_label = QLabel("状态: 初始化中...")
        layout.addWidget(self.status_label)

        # 数据显示区
        self.data_display = QTextEdit()
        self.data_display.setReadOnly(True)
        layout.addWidget(self.data_display)

        # 控制按钮
        btn_layout = QVBoxLayout()

        self.temp_btn = QPushButton("读取温度")
        self.temp_btn.clicked.connect(self.read_temperature)
        btn_layout.addWidget(self.temp_btn)

        self.humidity_btn = QPushButton("读取湿度")
        self.humidity_btn.clicked.connect(self.read_humidity)
        btn_layout.addWidget(self.humidity_btn)

        self.status_btn = QPushButton("读取状态")
        self.status_btn.clicked.connect(self.read_status)
        btn_layout.addWidget(self.status_btn)

        self.led_on_btn = QPushButton("开启LED")
        self.led_on_btn.clicked.connect(lambda: self.set_led(True))
        btn_layout.addWidget(self.led_on_btn)

        self.led_off_btn = QPushButton("关闭LED")
        self.led_off_btn.clicked.connect(lambda: self.set_led(False))
        btn_layout.addWidget(self.led_off_btn)

        layout.addLayout(btn_layout)

        # 错误信息显示
        self.error_label = QLabel()
        self.error_label.setStyleSheet("color: red;")
        layout.addWidget(self.error_label)

    def update_ui(self):
        # 更新状态
        if self.comm.ser and self.comm.ser.is_open:
            self.status_label.setText("状态: 已连接")
            self.status_label.setStyleSheet("color: green;")
        else:
            self.status_label.setText(f"状态: 未连接 ({self.comm.last_error or '未知错误'})")
            self.status_label.setStyleSheet("color: red;")

        # 显示队列数据
        data = self.comm.get_queued_data()
        if data:
            for item in data:
                if item['type'] == 'temperature':
                    text = f"[{item['timestamp']}] 温度: {item['value']}{item['unit']}"
                elif item['type'] == 'humidity':
                    text = f"[{item['timestamp']}] 湿度: {item['value']}{item['unit']}"
                elif item['type'] == 'status':
                    text = f"[{item['timestamp']}] 状态: {item['value']}"

                self.data_display.append(text)

    def read_temperature(self):
        temp = self.comm.read_sensor(0x01)
        if temp is not None:
            self.data_display.append(f"当前温度: {temp}°C")
        else:
            self.error_label.setText("读取温度失败")

    def read_humidity(self):
        humidity = self.comm.read_sensor(0x02)
        if humidity is not None:
            self.data_display.append(f"当前湿度: {humidity}%")
        else:
            self.error_label.setText("读取湿度失败")

    def read_status(self):
        status = self.comm.read_sensor(0x03)
        if status is not None:
            text = f"系统状态: 电压={status['voltage']}V, 电流={status['current']}A"
            self.data_display.append(text)
        else:
            self.error_label.setText("读取状态失败")

    def set_led(self, state):
        payload = bytes([1 if state else 0])
        if self.comm.send_command(0x10, payload):
            self.data_display.append(f"LED已{'开启' if state else '关闭'}")
        else:
            self.error_label.setText("控制LED失败")

    def closeEvent(self, event):
        self.comm.stop_worker()
        self.comm.close()
        event.accept()


if __name__ == "__main__":
    # 初始化通信模块
    comm = RA6M5Communicator()

    # 启动GUI
    app = QApplication(sys.argv)
    gui = RA6M5ControlGUI(comm)
    gui.show()
    sys.exit(app.exec_())