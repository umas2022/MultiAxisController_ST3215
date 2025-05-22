"""
高频指令控制，电机保持init位置
采集电机反馈保存到csv
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


# 数据采集
print("start collecting data ...")
data_buffer = []
start_time = time.perf_counter()
next_sample_time = start_time

for _ in range(100):  # 共采集 100 次
    t0 = time.perf_counter()

    dog12.move_all_init(1000, 50)  # 初始化电机位置

    # 采集数据
    positions = dog12.get_all_position_raw()
    loads = dog12.get_all_load()
    tempers = dog12.get_all_temper()

    # 计算采样时间
    t1 = time.perf_counter()
    timestamp = t0 - start_time  # 相对采样时间（秒）
    duration = t1 - t0  # 获取数据耗时（秒）

    data_buffer.append([timestamp, duration] + positions + loads + tempers)

    # 计算下一次采样点
    next_sample_time += 0.01  # 10ms 周期
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
