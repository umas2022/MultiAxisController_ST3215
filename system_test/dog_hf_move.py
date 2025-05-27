"""
高频指令控制，电机运动
狗
"""

import time
import csv
import sys

sys.path.append("..")
from MultiAxisSystem.DogController import DogController

# 初始化控制器
test_agent = DogController(serial_port="COM14")
test_agent.hardware_init()

# 在线检查
while True:
    time.sleep(1)
    print("servo online check ...")
    if test_agent.online_check():
        print("All motors are online.")
        break
    else:
        print("Some motors are offline. Please check the connections.")

# 参数设置
record_frequency = 50  # 采集频率（Hz）
record_period = 1 / record_frequency  # 采集周期（秒）

# 数据采集
data_buffer = []
start_time = time.perf_counter()
next_sample_time = start_time


# 初始化电机位置
pos_list_stand = [0, -256, 256, 0, -256, 256, 0, -256, 256, 0, -256, 256]
spd_list_stand = [1000] * 12
acc_list_stand = [50] * 12

test_agent.move_all_position_offset(pos_list_stand, spd_list_stand, acc_list_stand)

time.sleep(2)


with open("LH_FOOT_5cm_0_5Hz.csv", "r", encoding="utf-8") as file:
    reader = csv.reader(file)
    # 跳过表头
    for _ in range(1):
        next(reader)
    for row in reader:
        pos4 = -int(float(row[3]) / 3.14159 * 2048 * 2)
        pos5 = int(float(row[4]) / 3.14159 * 2048 * 2)
        pos6 = int(float(row[5]) / 3.14159 * 2048 * 2)
        pos_list = [0] * 3 + [pos4, pos5, pos6] + [0] * 6

        t0 = time.perf_counter()

        test_agent.move_all_position_offset(pos_list, [1000] * 12, [50] * 12)  # 移动到目标位置

        # 采集数据
        positions = []
        loads = []
        tempers = []
        # positions = test_agent.get_all_position_raw()
        # loads = test_agent.get_all_load()
        # tempers = test_agent.get_all_temper()

        # 计算采样时间
        t1 = time.perf_counter()
        timestamp = t0 - start_time  # 相对采样时间（秒）
        duration = t1 - t0  # 获取数据耗时（秒）

        data_buffer.append([timestamp, duration] + positions + loads + tempers)

        # 计算下一次采样点
        next_sample_time += record_period  # 10ms 周期
        sleep_time = next_sample_time - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)

# 写入 CSV 文件
with open("position_data.csv", mode="w", newline="") as file:
    writer = csv.writer(file)
    header = ["Time", "Duration"] + [f"Motor_{i+1}" for i in range(7)]
    writer.writerow(header)
    writer.writerows(data_buffer)

print("Data collection complete. Saved to position_data.csv")
