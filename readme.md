# st3215多轴运动系统python上位机

- 支持usb直连电机，或esp32串口
- 配套esp32下位机：https://github.com/umas2022/MultiAxisHandler_ST3215


## 项目结构

- MultiAxisSystem/
  - 多轴系统主函数，参见示例ControllerOne.py

- STServo_sdk/
  - st3215电机驱动sdk

- STServo_Python/
  - sdk附带的example，未来会删掉

- system_test/
  - 硬件系统性能测试脚本

- toolbox/
  - 基于MultiAxisSystem的工具包


## 待办
- MultiAxisSerial串口返回消息没有校验