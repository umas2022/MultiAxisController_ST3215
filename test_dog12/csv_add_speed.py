"""
dog12_hf_recorder.py记录了反馈速度，观察到反馈速度存在误差
此脚本根据反馈位置和时间计算速度，补充到csv文件尾部
"""

import pandas as pd
import numpy as np


def calculate_motor_velocities(input_csv, output_csv):
    # 读取CSV文件
    df = pd.read_csv(input_csv)

    # 提取时间列（单位：秒）
    time = df["Time"].values

    # 初始化速度DataFrame（与原始数据行数相同）
    velocities = pd.DataFrame()

    # 对每个电机位置计算速度
    for i in range(1, 13):
        pos_col = f"pos_{i}"
        vel_col = f"spd_cal_{i}"

        # 获取电机位置数据（单位：假设是编码器值或角度，具体单位需确认）
        positions = df[pos_col].values

        # 计算速度（差分位置 / 差分时间）
        # 注意：第一行的速度无法计算，设为NaN或0
        dt = np.diff(time)
        dp = np.diff(positions)
        speed = np.concatenate(([np.nan], dp / dt))  # 第一行速度设为NaN

        # 将速度添加到DataFrame
        velocities[vel_col] = speed

    # 将速度数据合并到原始DataFrame
    result_df = pd.concat([df, velocities], axis=1)

    # 保存到新的CSV文件
    result_df.to_csv(output_csv, index=False)
    print(f"结果已保存到: {output_csv}")


# 使用示例
input_csv = r"D:\workspace_local\ws_code\self\MultiAxisController_ST3215\system_test\test_output\lf_test_output.csv"  # 替换为你的输入CSV路径
output_csv = r"D:\workspace_local\ws_code\self\MultiAxisController_ST3215\system_test\test_output\lf_test_speed.csv"  # 输出CSV路径
calculate_motor_velocities(input_csv, output_csv)
