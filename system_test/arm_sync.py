"""
机械臂主从机同步运动
"""

import time
import csv
import sys

sys.path.append("..")
from MultiAxisSystem.ControllerArm import ControllerArm

# 初始化控制器
arm_master = ControllerArm(mode="usb", serial_port="COM12")
arm_slave = ControllerArm(mode="usb", serial_port="COM4")

arm_master.hardware_init()
arm_slave.hardware_init()

# 在线检查
while True:
    time.sleep(1)
    print("online check ...")
    if arm_master.online_check() and arm_slave.online_check():
        print("All motors are online.")
        break
    else:
        print("Some motors are offline. Please check the connections.")


arm_slave.move_all_absolute(arm_master.get_all_position(), [500] * 7, [50] * 7)
print("Slave arm initialized to master position.")
print("start sync ...")


while True:
    arm_slave.move_all_absolute(arm_master.get_all_position(), [2000] * 7, [0] * 7)
