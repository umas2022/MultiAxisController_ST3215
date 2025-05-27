"""
高频指令控制，采集电机反馈到csv，记录程序运行时间
https://www.yuque.com/u41016558/dgnm24/ebtkzsts82wgro0p#Q57RR
"""

import time
import csv
import sys

sys.path.append("..")
from MultiAxisSystem.ControllerArm import ControllerArm


# 初始化控制器
test_agent = ControllerArm(serial_port="COM9", mode="serial")  # 使用串口模式
# test_agent = ControllerArm(serial_port="COM15", mode="serial")
# test_agent = ControllerArm(mode="udp")  # 使用udp模式
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
total_step = 1  # 采集步数
total_duration = 1.0  # 总采集时间（秒）
record_period = total_duration / total_step  # 采集周期（秒）
record_frequency = 1 / record_period  # 采集频率（Hz）

# 位置矩阵
# target_pos_offset = [0, 0, 0, 0, 0, 0, 0]
target_pos_offset = [100, -100, -100, -100, -100, 100, 100]  # 目标位移（偏移量）
step_increment = [x / total_step for x in target_pos_offset]
target_pos_offset_matrix = [[int(step_increment[i] * step) for i in range(7)] for step in range(1, total_step + 1)]
# 速度矩阵
# spd_list = [int(100 / total_duration)] * 7
spd_list = [1000] * 7
# 加速度矩阵
acc_list = [0] * 7


# 初始化电机位置
test_agent.move_all_init(100, 0)
time.sleep(2)


# 数据采集
print("start collecting data ...")
data_buffer = []
start_time = time.perf_counter()
next_sample_time = start_time


for current_step in range(total_step):  # 共采集 100 次
    t0 = time.perf_counter()

    test_agent.move_all_offset(target_pos_offset_matrix[current_step], spd_list, acc_list)  # 移动到目标位置

    # 采集数据
    # positions = []
    # loads = []
    # tempers = []
    positions = test_agent.get_all_position()
    loads = test_agent.get_all_load()
    tempers = test_agent.get_all_temper()

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
with open(r"D:\workspace_local\ws_code\test\test_data\position_data.csv", mode="w", newline="") as file:
    writer = csv.writer(file)
    header = ["Time", "Duration"] + [f"Motor_{i+1}" for i in range(7)]
    writer.writerow(header)
    writer.writerows(data_buffer)

print("Data collection complete. Saved to position_data.csv")
