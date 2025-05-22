"""
运动控制
"""

import time
import csv
import sys

sys.path.append("..")
from MultiAxisSystem.DogController import DogController

# 初始化控制器
dog12 = DogController(serial_port="COM20")
dog12.hardware_init()

# 在线检查
while True:
    time.sleep(1)
    print("servo online check ...")
    if dog12.online_check():
        print("All motors are online.")
        break
    else:
        print("Some motors are offline. Please check the connections.")


# 初始化电机位置
# dog12.move_all_init(1000, 50)

time.sleep(2)

pos_list_stand = [64, -256, 256, 64, -256, 256, 64, -256, 256, 64, -256, 256]
spd_list_stand = [1000] * 12
acc_list_stand = [50] * 12

dog12.move_all_position_offset(pos_list_stand, spd_list_stand, acc_list_stand)
