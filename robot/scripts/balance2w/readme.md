# balance2w

两轮平衡车调试脚本，当前按你的现有链路工作：

- 电机控制：`ESP32 -> 蓝牙串口 COM8`
- IMU：`电脑直连 USB 串口 COM3`
- 电机 ID：`1` 和 `2`

## 运行前提

- ESP32 已烧录正常主固件
- `COM8` 能扫描到电机 `1/2`
- IMU 直连电脑时可在 `COM3` 读到姿态
- 小车先架空，不要直接落地

## 先验命令

先确认 IMU：

```bash
python MultiAxisController_ST3215\toolbox\imu_read.py --mode usb --port COM3
```

再确认电机：

```bash
python MultiAxisController_ST3215\toolbox\motor_id_scann.py --mode serial --port COM8 --start-id 1 --end-id 3
```

## 当前确认过的安装配置

你这台车当前已经确认：

- 俯仰轴：`pitch_axis=0`
- 俯仰方向：`pitch_sign=-1`

现象依据：

- 原先 `pitch_axis=1` 时，前后倾轮子不动，左右倾才有反应
- 改成 `pitch_axis=0` 后，前后倾开始有反应
- 再改成 `pitch_sign=-1` 后，前倾向前追、后倾向后追，方向正确

因此当前脚本默认值已经同步成：

- `pitch_axis=0`
- `pitch_sign=-1`

## 启动命令

当前推荐从低风险参数开始：

```bash
python MultiAxisController_ST3215\robot\scripts\balance2w\balance_serial.py --motor-mode serial --motor-port COM8 --imu-mode usb --imu-port COM3 --max-speed 400 --kp 18 --kd 0.35
```

## 关键参数

- `--pitch-axis`
  - 俯仰轴索引，候选 `0/1/2`
  - 当前默认 `0`
- `--pitch-sign`
  - 俯仰方向符号
  - 当前默认 `-1`
- `--disable-auto-zero-target`
  - 默认会把启动瞬间姿态当成平衡目标
  - 如果你要手动指定平衡零点，再关闭它并配合 `--target-pitch-deg`
- `--kp --kd`
  - 平衡环比例和微分
  - 先从小值开始
- `--max-speed`
  - 限速，第一次建议不超过 `400`

## 支架调试判断标准

先看日志里的：

- `pitch_deg`
- `pitch_rate_dps`
- `left_cmd`
- `right_cmd`

目标现象：

1. 前后倾时，`pitch_deg` 要明显变化
2. 前倾时，轮子应向前追车
3. 后倾时，轮子应向后追车
4. 直立附近时，`left_cmd/right_cmd` 应接近 `0`

## 建议调参顺序

### 第一步：继续支架调 `kp`

先从：

```bash
python MultiAxisController_ST3215\robot\scripts\balance2w\balance_serial.py --motor-mode serial --motor-port COM8 --imu-mode usb --imu-port COM3 --max-speed 800 --kp 60 --kd 0.35
```

如果追车反应偏软，逐步加：

- `kp 18 -> 24`
- `kp 24 -> 30`
- `kp 30 -> 36`

判断标准：

- `kp` 太小：前后倾时追车明显滞后，扶着车时“软”
- `kp` 太大：轮子开始来回冲、明显发抖

### 第二步：再调 `kd`

当 `kp` 已经能明显追车后，再看是否抖动：

- 如果容易来回摆，增加 `kd`
- 推荐尝试：`0.35 -> 0.5 -> 0.7 -> 0.9`

判断标准：

- `kd` 太小：回摆明显
- `kd` 太大：动作发闷，响应迟钝

### 第三步：再降限速准备落地

在支架上方向和追车都正常后，先把落地初始限速收紧：

```bash
--max-speed 250
```

必要时也可以先把 `kp` 退一点，例如从 `30` 退回 `24`

## 什么时候可以落地

只有在以下条件同时满足时，才建议从支架转到地面：

- 前后倾能稳定触发轮子追车
- 方向完全正确
- 直立附近不会持续大幅输出
- 支架上没有明显高频抖动或瞬间打满

第一次落地仍然建议：

- `max_speed <= 250`
- 保持人手扶持
- 预备随时断电或停止脚本

## 落地后的第一轮目标

第一次落地不要追求“完全松手站稳”，只看三件事：

1. 会不会立刻朝一个方向暴冲
2. 会不会马上高频抖动
3. 扶持状态下能不能明显给出纠偏力

如果出现暴冲：

- 先停机
- 优先减小 `kp`
- 确认启动时车体姿态是否接近直立

如果出现高频抖动：

- 稍减 `kp`
- 或增加 `kd`
