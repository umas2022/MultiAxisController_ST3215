"""
终端打印电机反馈
"""

import time
import csv
import sys

sys.path.append("..")
from MultiAxisSystem.DogController import DogController

# 初始化控制器
dog12 = DogController(serial_port="COM4")
dog12.hardware_init()

# 在线检查
while True:
    time.sleep(1)
    print("master arm online check ...")
    if dog12.online_check():
        print("All motors are online.")
        break
    else:
        print("Some motors are offline. Please check the connections.")

dog12.move_all_init(1000, 50)  # 初始化电机位置

while True:
    positions = dog12.get_all_position_raw()
    loads = dog12.get_all_load()
    # print("positions:", positions)
    # print("loads:", loads)
    print("temperatures:", dog12.get_all_temper())
    time.sleep(1)
