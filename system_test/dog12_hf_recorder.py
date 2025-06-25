"""
狗12高频指令控制，反馈采集
"""

import time
import csv
import sys
from itertools import islice

sys.path.append("..")
from MultiAxisSystem.ControllerDog12F import ControllerDog12F

step_import_csv = "lf_test.csv"
data_output_csv = "output/position_data.csv"

# 参数设置
record_frequency = 50  # 采集频率（Hz）
record_period = 1 / record_frequency  # 采集周期（秒）
downsample_csv = 10  # csv降采样因子
total_samples = 1000  # 总采样点数

# 初始化控制器
test_agent = ControllerDog12F(mode="usb", serial_port="COM21")
test_agent.hardware_init()

# 在线检查
while True:
    time.sleep(1)
    print("servo online check ...")
    if test_agent.online_check():
        print("All motors are online.")
        break


# 数据采集
data_buffer = []
start_time = time.perf_counter()
next_sample_time = start_time

# 初始化姿态
test_agent.move_all_init(1000, 50)
time.sleep(2)
test_agent.stand_up()
time.sleep(2)
print("start recording ...")


with open(step_import_csv, "r", encoding="utf-8") as file:
    reader = csv.reader(file)
    # 跳过表头
    for _ in range(1):
        next(reader)
    # 用 islice 跳过非目标行（避免内存加载全部数据）
    downsampled_rows = islice(reader, 0, None, downsample_csv)
    for i, row in enumerate(downsampled_rows):
        if total_samples is not None and i >= total_samples:  # 新增终止条件
            break
        pos1 = int(float(row[0]) / 3.14159 * 2048 * 2)
        pos2 = int(float(row[1]) / 3.14159 * 2048 * 2)
        pos3 = int(float(row[2]) / 3.14159 * 2048 * 2)
        pos4 = -int(float(row[3]) / 3.14159 * 2048 * 2)
        pos5 = int(float(row[4]) / 3.14159 * 2048 * 2)
        pos6 = int(float(row[5]) / 3.14159 * 2048 * 2)
        pos7 = int(float(row[6]) / 3.14159 * 2048 * 2)
        pos8 = -int(float(row[7]) / 3.14159 * 2048 * 2)
        pos9 = int(float(row[8]) / 3.14159 * 2048 * 2)
        pos10 = int(float(row[9]) / 3.14159 * 2048 * 2)
        pos11 = -int(float(row[10]) / 3.14159 * 2048 * 2)
        pos12 = int(float(row[11]) / 3.14159 * 2048 * 2)
        pos_list = [pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8, pos9, pos10, pos11, pos12]

        offset_list = [x + y for x, y in zip(pos_list, test_agent.posture_stand)]

        t0 = time.perf_counter()

        test_agent.move_all_offset(offset_list, [1000] * 12, [50] * 12)  # 移动到目标位置

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
with open(data_output_csv, mode="w", newline="") as file:
    writer = csv.writer(file)
    header = ["Time", "Duration"] + [f"pos_{i+1}" for i in range(12)] + [f"load_{i+1}" for i in range(12)] + [f"temper_{i+1}" for i in range(12)]
    writer.writerow(header)
    writer.writerows(data_buffer)

print("Data collection complete. Saved to hf_data.csv")
