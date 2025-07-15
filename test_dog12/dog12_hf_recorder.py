"""
狗12高频指令控制，反馈采集
"""

import time
import csv
import sys
from itertools import islice
import os
from pathlib import Path

sys.path.append("..")
from MultiAxisSystem.ControllerDog12F import ControllerDog12F


# csv路径
# csv_import_dir = Path(r"D:\workspace_local\ws_code\self\MultiAxisController_ST3215\system_test\test_data")
# csv_output_dir = Path(r"D:\workspace_local\ws_code\self\MultiAxisController_ST3215\system_test\test_output")
csv_import_dir = Path(r"D:\workspace_active\diy003_servo_dog_12dof\analysis\ik_csv")
csv_output_dir = Path(r"D:\workspace_active\diy003_servo_dog_12dof\analysis\ik_csv_output")
csv_output_dir.mkdir(parents=True, exist_ok=True)
csv_files = list(csv_import_dir.glob("*.csv"))
total_files = len(csv_files)

# 初始化控制器
test_agent = ControllerDog12F(mode="usb", serial_port="COM28")
test_agent.hardware_init()

# 在线检查
while True:
    time.sleep(1)
    print("servo online check ...")
    if test_agent.online_check():
        print("All motors are online.")
        break

# 初始化姿态
test_agent.move_all_init(1000, 50)
time.sleep(2)
test_agent.stand_up()
time.sleep(2)
print("start recording ...")


def run_with_csv(csv_file, output_file):
    print(f"[{i}/{total_files}] Processing {csv_file.name}...", flush=True)

    # 参数设置
    # csv行数24000，频率100Hz，总时间24000/100=240s；动作频率0.05，动作步数12，总时间12/0.05=240s
    # 降采样2，实际总采样点数12000，采集频率50Hz，总时间12000/50=240秒；动作频率12/240=0.05Hz
    record_frequency = 50  # 采集频率（Hz）
    downsample_csv = 2  # csv降采样因子(每n行采样一次)
    total_samples = 24000 / downsample_csv  # 实际总采样点数
    # total_samples = 240

    # 数据采集
    data_buffer = []
    start_time = time.perf_counter()
    next_sample_time = start_time

    with open(csv_file, "r", encoding="utf-8") as file:
        reader = csv.reader(file)
        # 跳过表头
        for _ in range(1):
            next(reader)
        # 用 islice 跳过非目标行（避免内存加载全部数据）
        downsampled_rows = islice(reader, 0, None, downsample_csv)
        for index, row in enumerate(downsampled_rows):
            if total_samples is not None and index >= total_samples:  # 新增终止条件
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
            raw_position = [0] * 12
            raw_speed = [0] * 12
            raw_load = [0] * 12
            raw_temper = [0] * 12
            raw_position = test_agent.get_all_position()
            # raw_speed = test_agent.get_all_speed()
            raw_load = test_agent.get_all_load()
            raw_temper = test_agent.get_all_temper()
            # raw_position, raw_load, raw_temper = test_agent.ctrl.get_all_position_load_temper()

            # 计算采样时间
            t1 = time.perf_counter()
            timestamp = t0 - start_time  # 相对采样时间（秒）
            duration = t1 - t0  # 获取数据耗时（秒）

            data_buffer.append([timestamp, duration] + raw_position + raw_speed + raw_load + raw_temper)

            # 计算下一次采样点
            next_sample_time += 1 / record_frequency  # 10ms 周期
            sleep_time = next_sample_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

            if index % 100 == 0:
                print(f"[{i}/{total_files}] Processing {csv_file.name}... [{index}/{int(total_samples)}]", flush=True)

    # 写入 CSV 文件
    with open(output_file, mode="w", newline="") as file:
        writer = csv.writer(file)
        header = ["Time", "Duration"] + [f"pos_{i+1}" for i in range(12)] + [f"speed_{i+1}" for i in range(12)] + [f"load_{i+1}" for i in range(12)] + [f"temper_{i+1}" for i in range(12)]
        writer.writerow(header)
        writer.writerows(data_buffer)


# 处理循环
for i, csv_file in enumerate(csv_files, start=1):
    output_filename = f"{csv_file.stem}_output.csv"
    output_path = csv_output_dir / output_filename
    run_with_csv(csv_file, output_path)


print("Data collection complete")
