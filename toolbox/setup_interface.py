"""
st3215电机初始化工具，配合固件pio_serial_setup
pip install pyqt5 pyserial

此脚本不再更新，推荐使用飞特官方调试软件设定零点
"""

import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QTextEdit,
    QComboBox,
    QLabel,
    QLineEdit,
)
from PyQt5.QtCore import QTimer


class MotorInitToolGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Motor Init Tool GUI")
        self.resize(600, 600)

        self.serial_port = None

        # 串口选择
        self.port_selector = QComboBox()
        self.refresh_ports()

        self.connect_button = QPushButton("连接串口")
        self.connect_button.clicked.connect(self.toggle_connection)

        self.log = QTextEdit()
        self.log.setReadOnly(True)

        layout = QVBoxLayout()

        # ➤ 串口选择区
        port_layout = QHBoxLayout()
        port_layout.addWidget(self.port_selector)
        port_layout.addWidget(self.connect_button)
        layout.addLayout(port_layout)

        # ➤ 按钮+输入框区域
        self.create_command_ui(layout)

        # ➤ 日志区
        layout.addWidget(QLabel("日志输出:"))
        layout.addWidget(self.log)

        self.setLayout(layout)

        # 串口监听定时器
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial)

    def refresh_ports(self):
        self.port_selector.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_selector.addItem(port.device)

    def toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.connect_button.setText("连接串口")
            self.timer.stop()
            self.log.append("已断开串口连接")
        else:
            port_name = self.port_selector.currentText()
            try:
                self.serial_port = serial.Serial(port_name, baudrate=115200, timeout=0.1)
                self.connect_button.setText("断开串口")
                self.timer.start(100)
                self.log.append(f"已连接到 {port_name}")
            except Exception as e:
                self.log.append(f"连接失败: {e}")

    def create_command_ui(self, parent_layout):
        # ➤ 1. PING
        ping_layout = QHBoxLayout()
        self.ping_input = QLineEdit()
        self.ping_input.setPlaceholderText("电机ID")
        ping_button = QPushButton("PING电机")
        ping_button.clicked.connect(self.send_ping_command)
        ping_layout.addWidget(ping_button)
        ping_layout.addWidget(self.ping_input)
        parent_layout.addLayout(ping_layout)

        # ➤ 2. SET_ID
        setid_layout = QHBoxLayout()
        self.oldid_input = QLineEdit()
        self.oldid_input.setPlaceholderText("旧ID")
        self.newid_input = QLineEdit()
        self.newid_input.setPlaceholderText("新ID")
        setid_button = QPushButton("设置ID")
        setid_button.clicked.connect(self.send_setid_command)
        setid_layout.addWidget(setid_button)
        setid_layout.addWidget(self.oldid_input)
        setid_layout.addWidget(self.newid_input)
        parent_layout.addLayout(setid_layout)

        # ➤ 3. SET_ZERO
        setzero_layout = QHBoxLayout()
        self.setzero_input = QLineEdit()
        self.setzero_input.setPlaceholderText("电机ID")
        setzero_button = QPushButton("设置中点")
        setzero_button.clicked.connect(self.send_setzero_command)
        setzero_layout.addWidget(setzero_button)
        setzero_layout.addWidget(self.setzero_input)
        parent_layout.addLayout(setzero_layout)

        # ➤ 4. TEST
        test_layout = QHBoxLayout()
        self.test_input_id = QLineEdit()
        self.test_input_id.setPlaceholderText("电机ID")
        self.test_input_direct = QLineEdit()
        self.test_input_direct.setPlaceholderText("转动方向(0/1)")
        self.test_input_step = QLineEdit()
        self.test_input_step.setPlaceholderText("目标步数")
        test_button = QPushButton("转动测试")
        test_button.clicked.connect(self.send_run_command)
        test_layout.addWidget(test_button)
        test_layout.addWidget(self.test_input_id)
        test_layout.addWidget(self.test_input_direct)
        test_layout.addWidget(self.test_input_step)
        parent_layout.addLayout(test_layout)

    def send_ping_command(self):
        motor_id = self.ping_input.text().strip()
        if motor_id:
            # self.send_text_command(f"PING {motor_id}")
            self.send_byte_command([0x01, 0x01, int(motor_id)])

    def send_setid_command(self):
        old_id = self.oldid_input.text().strip()
        new_id = self.newid_input.text().strip()
        if old_id and new_id:
            self.send_byte_command([0x02, 0x02, int(old_id), int(new_id)])

    def send_setzero_command(self):
        motor_id = self.setzero_input.text().strip()
        if motor_id:
            self.send_byte_command([0x03, 0x01, int(motor_id)])

    def send_run_command(self):
        motor_id = self.test_input_id.text().strip()
        direction = self.test_input_direct.text().strip()
        step = self.test_input_step.text().strip()
        if not motor_id or not direction or not step:
            self.log.append("请填写电机ID、转动方向和目标步数！")
            return
        step_low = int(step) & 0xFF  # 低8位
        step_high = (int(step) >> 8) & 0xFF  # 高8位
        self.send_byte_command([0x04, 0x04, int(motor_id), int(direction), int(step_high), int(step_low)])

    def send_byte_command(self, byte_command):
        if not self.serial_port or not self.serial_port.is_open:
            self.log.append("串口未连接，无法发送指令！")
            return
        try:
            checksum = sum(byte_command) & 0xFF
            frame = bytearray([0xAA] + byte_command + [checksum])
            self.serial_port.write(frame)
            self.log.append(f"发送指令: {[hex(b) for b in frame]}")
        except Exception as e:
            self.log.append(f"发送失败: {e}")

    def read_serial(self):
        if self.serial_port and self.serial_port.in_waiting:
            try:
                data = self.serial_port.read(self.serial_port.in_waiting)
                # text = data.decode("utf-8", errors="replace")
                # self.log.append(text)
                hex_str = " ".join(f"{byte:02X}" for byte in data)
                self.log.append(hex_str)
            except Exception as e:
                self.log.append(f"读取失败: {e}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MotorInitToolGUI()
    window.show()
    sys.exit(app.exec_())
