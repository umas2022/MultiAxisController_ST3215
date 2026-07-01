# Toolbox

面向 ST3215、IMU 和 ELRS 硬件的独立调试工具。电机工具均在脚本顶部集中配置连接模式、COM 口和电机参数。

## STServo 官方示例替代关系

| 原官方示例 | Toolbox 替代工具 | 覆盖功能 |
| --- | --- | --- |
| `ping.py` | `motor_id_scann.py` | Ping 单个或一段 ID |
| `read.py` | `motor_read_status.py` | 读取位置和速度，并额外读取负载、温度 |
| `write.py` | `motor_move.py` | 单电机绝对位置、速度和加速度控制 |
| `read_write.py` | `motor_move.py` | 写入目标并轮询位置直到运动完成 |
| `reg_write.py` | `motor_move.py` | 多电机注册写入后统一 Action，同步开始运动 |
| `sync_write.py` | `motor_move.py` | 批量下发多电机目标位置 |
| `sync_read.py` | `motor_read_status.py` | 批量读取多电机状态 |
| `sync_read_write.py` | `motor_move.py` + `motor_read_status.py` | 多电机运动与反馈读取 |
| `wheel.py` | `motor_wheel_test.py` | 轮式模式、正反转、停止和退出保护 |

## 电机工具

- `motor_id_scann.py`：扫描在线电机 ID
- `motor_read_position.py`：读取单个电机位置
- `motor_read_status.py`：连续读取多电机位置、速度、负载和温度
- `motor_move.py`：单电机或多电机同步位置控制
- `motor_wheel_test.py`：轮式模式测试，默认由安全开关禁用
- `motor_set_id.py`：修改电机 ID
- `motor_set_2048.py`：将当前位置校准为中位 2048

涉及运动或参数写入的工具应先架空设备，并确认总线上电机 ID 与脚本配置一致。
