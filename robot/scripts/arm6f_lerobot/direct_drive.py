# [2462, 1834, 2530, 2011, 2084]
"""
发布当前状态
"""

import time
import sys
import os

# Add the robot directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))
from src.controller.ControllerArm6F import ControllerArm

# 初始化控制器
# arm6f = ControllerArm(mode="usb", serial_port="COM20") 
arm6f = ControllerArm(mode="usb", serial_port="COM20") 

# 初始化硬件 (opens the serial port)
try:
    if not arm6f.hardware_init():
        print("Failed to initialize hardware connection")
        sys.exit(1)
except Exception as e:
    print(f"Error initializing hardware: {e}")
    print("Please check if the device is connected and the correct COM port is specified.")
    sys.exit(1)

# 在线检查
while True:
    print("arm6f online check ...")
    arm6f_online = arm6f.online_check()
    if arm6f_online:
        break
    print("\n")
    time.sleep(1)
print("arm6f online")



while True:
    # js_pos = arm6f.get_all_position()
    # print("arm6f position:", js_pos)

    arm6f.move_all_absolute(pos=[2447, 1962, 2779, 1996, 2076, 2027], spd= [2000] * 6, acc= [0] * 6)

    time.sleep(1)
