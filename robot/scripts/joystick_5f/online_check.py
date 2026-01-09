"""
发布摇杆状态
"""

import time
import sys
import os

# Add the robot directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))
from src.controller.ControllerJoyStick5F import ControllerJoyStick

# 初始化控制器
# You may need to change the serial port to match your actual device
joy_stick = ControllerJoyStick(mode="usb", serial_port="/dev/ttyACM0")  # Changed from COM31 to COM3 as an example

# 初始化硬件 (opens the serial port)
try:
    if not joy_stick.hardware_init():
        print("Failed to initialize hardware connection")
        sys.exit(1)
except Exception as e:
    print(f"Error initializing hardware: {e}")
    print("Please check if the device is connected and the correct COM port is specified.")
    sys.exit(1)

# 在线检查
while True:
    print("joy_stick online check ...")
    joy_stick_online = joy_stick.online_check()
    if joy_stick_online:
        break
    print("\n")
    time.sleep(1)
print("joy_stick online")



while True:
    js_pos = joy_stick.get_all_position()
    print("joy_stick position:", js_pos)
    time.sleep(1)
