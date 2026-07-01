# teleop_2axis_h3106

H3106 两轴遥操作控制器，使用两个 ST3215 电机读取两个摇杆轴的位置。

固定硬件配置：

- USB 串口：`COM16`
- 电机 ID：`1`、`2`
- 中点：`2048`
- 每圈编码：`4096`
- 每个轴的机械范围：约 `-45°～+45°`
- `-45°` 对应位置：`2048 - 4096 × 45 / 360 = 1536`

## 初始化流程

调用 `init()` 后，控制器会：

1. 打开 COM16 并检查两个电机在线状态；
2. 开启两个电机的扭矩；
3. 同步运动到位置 `1536`，即约 `-45°`；
4. 等待两个电机到位；
5. 关闭扭矩，释放两个摇杆供手动操作。

初始化过程中即使移动失败或超时，也会尝试释放已经开启扭矩的电机。

## Python 接口

```python
from robot.systems.teleop_2axis_h3106 import Teleop2AxisController

controller = Teleop2AxisController()
if not controller.init():
    raise RuntimeError("H3106 initialization failed")

position_1, position_2 = controller.read_positions()
print(position_1, position_2)
```

对外读取接口只有 `read_positions()`，返回两个电机的原始编码器位置。

## 连续读取示例

```powershell
python -m robot.systems.teleop_2axis_h3106.apps.read_state
```
