"""
原始文件中每个csv仅包含单腿数据，即12列中仅有3列有效
此脚本将多个单腿csv文件合并为一个包含所有腿部数据的csv文件
"""

import os
import pandas as pd
import re
from collections import defaultdict

# ===== 你可以在这里修改输入和输出目录 =====
input_dir = r"D:\workspace_active\diy003_servo_dog_12dof\analysis\ik_csv"  # 你的原始单腿文件目录
output_dir = r"D:\workspace_active\diy003_servo_dog_12dof\analysis\ik_csv_merge"  # 合并后文件输出目录
# ============================================

# 创建输出目录（如果不存在）
os.makedirs(output_dir, exist_ok=True)

# 找出输入目录中所有csv文件
csv_files = [f for f in os.listdir(input_dir) if f.endswith(".csv")]

# 用于按参数分组
grouped_files = defaultdict(dict)

# 匹配模式
pattern = re.compile(r"(LF|LH|RF|RH)_FOOT_ellipse_(X\d+cm_Y\d+cm_H\d+cm_12dir_0_1Hz)")

# 解析并分类
for file in csv_files:
    match = pattern.match(file)
    if match:
        leg, key = match.groups()
        grouped_files[key][leg] = file

# 12关节名称
joint_names = ["LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE", "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"]

# 合并处理
for key, legs in grouped_files.items():
    if all(leg in legs for leg in ["LF", "LH", "RF", "RH"]):
        # 读取数据
        dfs = {leg: pd.read_csv(os.path.join(input_dir, legs[leg])) for leg in ["LF", "LH", "RF", "RH"]}

        # 创建空DF
        merged_df = pd.DataFrame(0.0, index=dfs["LF"].index, columns=joint_names)

        # 插入各自数据
        merged_df[["LF_HAA", "LF_HFE", "LF_KFE"]] = dfs["LF"][["LF_HAA", "LF_HFE", "LF_KFE"]]
        merged_df[["LH_HAA", "LH_HFE", "LH_KFE"]] = dfs["LH"][["LH_HAA", "LH_HFE", "LH_KFE"]]
        merged_df[["RF_HAA", "RF_HFE", "RF_KFE"]] = dfs["RF"][["RF_HAA", "RF_HFE", "RF_KFE"]]
        merged_df[["RH_HAA", "RH_HFE", "RH_KFE"]] = dfs["RH"][["RH_HAA", "RH_HFE", "RH_KFE"]]

        # 写入文件
        output_filename = f"MERGED_ellipse_{key}.csv"
        merged_df.to_csv(os.path.join(output_dir, output_filename), index=False)
        print(f"✔ 已合并: {output_filename}")
    else:
        print(f"⚠ 缺失腿部数据: {key} -> 已有: {list(legs.keys())}")
