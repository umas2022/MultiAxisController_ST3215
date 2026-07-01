# ST3215 多轴运动控制系统

基于 Python 的 ST3215 多轴运动系统上位机，适用于机械臂、机器狗、两轮平衡车、夹爪和遥操作摇杆等设备。

支持以下通信方式：

- USB 转串口直连 ST3215 总线
- 有线串口或蓝牙串口连接 ESP32 下位机
- UDP 连接 ESP32 下位机

相关项目与工具：

- [ESP32 下位机](https://github.com/umas2022/MultiAxisHandler_ST3215)
- [新版 ESP32 下位机（调试中）](https://github.com/umas2022/MultiAxisController_ST3215_esp32)
- [飞特官方调试工具](https://gitee.com/ftservo/fddebug)

## 架构

项目分为可复用驱动和具体运动系统两部分：

```text
robot ───────► multiaxis_driver
toolbox ─────► multiaxis_driver
```

- `multiaxis_driver` 不依赖任何具体机器人，可以单独安装到其他项目。
- `robot` 保存关节配置、控制逻辑和应用入口。
- `toolbox` 直接使用驱动包进行硬件诊断，不依赖机器人代码。

## 安装

要求 Python 3.9 或更高版本。建议在虚拟环境中安装：

```powershell
pip install -r requirements.txt
```

`requirements.txt` 会以开发模式安装本仓库中的驱动包。若只需要在其他项目中使用驱动，可以单独执行：

```powershell
pip install -e .\drivers
```

## 快速开始

### 使用硬件工具

工具脚本位于 `toolbox/`，可用于扫描电机、修改 ID、校准中位和读取位置：

```powershell
python toolbox\motor_id_scann.py
python toolbox\motor_read_position.py
python toolbox\motor_read_status.py
python toolbox\motor_move.py
python toolbox\motor_wheel_test.py
python toolbox\motor_set_id.py
python toolbox\motor_set_2048.py
```

执行会修改电机配置的脚本前，请确认总线上只连接了目标电机，并检查脚本中的串口、ID 和连接模式。
各工具说明及原 STServo SDK 示例的替代关系见 `toolbox/readme.md`。

### 运行机器人应用

从仓库根目录以模块方式运行应用：

```powershell
python -m robot.systems.arm_6axis_lerobot.apps.online_check
python -m robot.systems.wheeled_2axis_d0503.apps.balance_serial --help
```

每套设备的接线、端口和运行说明见对应系统目录下的 `readme.md`。

### 在代码中使用驱动

```python
from multiaxis_driver.motor import MotorConfig, MultiAxisUSB

driver = MultiAxisUSB("COM5")
driver.motors_list = [
    MotorConfig(id=1, min=0, max=4096, init=2048, reverse=False),
]

if driver.hardware_init():
    print("Motor driver initialized")
```

## 目录结构

```text
.
├─ drivers/                       # 可独立安装的硬件驱动包
│  └─ src/multiaxis_driver/
│     ├─ motor/                   # ST3215 USB、Serial、UDP 驱动
│     ├─ imu/                     # IMU 驱动
│     └─ elrs/                    # ELRS/CRSF 驱动
├─ robot/
│  ├─ common/                    # 机器人侧共享工具
│  └─ systems/                   # 按具体运动系统组织
│     └─ <system>/
│        ├─ controller.py         # 系统控制器与关节配置
│        ├─ apps/                 # 启动、测试和控制入口
│        └─ readme.md             # 系统说明
├─ toolbox/                       # 通用硬件调试工具
└─ docs/                          # 项目文档
```

当前包含的运动系统：

- `arm_6axis_lerobot`：六轴 LeRobot 机械臂
- `arm_7axis_aiot7dof_d0702`：D0702 七轴 AIOT 机械臂
- `teleop_8axis_realman_h2608`：H2608 RealMan 八轴遥操作器
- `wheeled_2axis_d0503`：D0503 双轮两轴平衡车
- `dog_4axis_d0611`、`dog_8axis_d0612`、`dog_12axis_d0405`：不同轴数的四足机器人
- `dog_12axis_spider_d1002`：D1002 十二轴蜘蛛布局四足机器人
- `teleop_2axis_h3106`：H3106 两轴遥操作器
- `teleop_5axis_h2631`：H2631 五轴遥操作器
- `actuator_1axis_example`：单执行器控制示例
- `hand_6axis_d0801`：D0801 六轴手部系统占位目录

## 开发约定

- 驱动层只能处理硬件通信和协议，不得导入 `robot`。
- 电机 ID、方向、限位、零点和运动算法放在对应的 `robot/systems/<system>/` 中。
- 通用硬件诊断功能放在 `toolbox/`。
- 新增运动系统时，将控制器、应用入口和说明文档放在同一个系统目录中。

## 已知问题

- 串口和 UDP 返回消息尚未完成完整校验。
- 下位机串口消息最大为 64 字节，超过后可能丢包。
