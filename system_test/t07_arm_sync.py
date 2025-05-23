""" """

import time
import csv
import sys

sys.path.append("..")
from MultiAxisSystem.ArmController import ArmController

# 初始化控制器
arm_master = ArmController(mode="usb", serial_port="COM12")
arm_slave = ArmController(mode="usb", serial_port="COM4")

arm_master.hardware_init()
arm_slave.hardware_init()

# 在线检查
while True:
    time.sleep(1)
    print("master arm online check ...")
    if arm_master.online_check():
        print("All motors are online.")
        break
    else:
        print("Some motors are offline. Please check the connections.")


# arm_master.move_all_init(1000, 50)
arm_slave.move_all_init(1000, 50)


while True:

    arm_slave.move_all_absolute(arm_master.get_all_position_raw(), [1000] * 7, [0] * 7)
