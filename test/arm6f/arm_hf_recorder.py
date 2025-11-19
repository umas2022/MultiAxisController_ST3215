"""
6d机械臂
高频指令控制，采集电机反馈到csv，记录程序运行时间
https://www.yuque.com/u41016558/dgnm24/ebtkzsts82wgro0p#Q57RR
"""

import time
import csv
import sys

sys.path.append("..")
from MultiAxisSystem.ControllerArm6F import ControllerArm


# 初始化控制器
# test_agent = ControllerArm(serial_port="COM16", mode="serial")
test_agent = ControllerArm(mode="udp")
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
record_frequency = 100  # 采集频率（Hz）
total_duration = 1.0  # 总采集时间（秒）
target_step = 100  # 转动目标步数

# 总采集步数
total_step = int(total_duration * record_frequency)
# 采集周期（秒）
record_period = total_duration / total_step
# 6d目标位置
target_pos_offset = [target_step] * 6
# 6d单次微分增量
step_increment = [x / total_step for x in target_pos_offset]
# 微分增量矩阵
target_pos_offset_matrix = [[int(step_increment[i] * step) for i in range(6)] for step in range(1, total_step + 1)]
# 6d速度向量
spd_list = [1000] * 6
# 6d加速度向量
acc_list = [0] * 6


# 初始化电机位置
print("init moter position ...")
test_agent.move_all_init(1000, 0)
time.sleep(1)


# 数据采集
print("start collecting data ...")
data_buffer = []
start_time = time.perf_counter()
next_sample_time = start_time


for current_step in range(total_step):  # 共采集 100 次
    t0 = time.perf_counter()

    test_agent.move_all_offset(target_pos_offset_matrix[current_step], spd_list, acc_list)  # 移动到目标位置

    # 采集数据
    positions = []
    loads = []
    tempers = []
    # positions = test_agent.get_all_position()
    # loads = test_agent.get_all_load()
    # tempers = test_agent.get_all_temper()
    positions, loads, tempers = test_agent.ctrl.get_all_position_load_temper()

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
with open(r"output/arm.csv", mode="w", newline="") as file:
    writer = csv.writer(file)
    header = ["Time", "Duration"] + [f"pos_{i+1}" for i in range(6)] + [f"load_{i+1}" for i in range(6)] + [f"temper_{i+1}" for i in range(6)]
    writer.writerow(header)
    writer.writerows(data_buffer)

print("Data collection complete. Saved to position_data.csv")
