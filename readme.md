# st3215多轴运动系统python上位机

- 支持上位机usb直连电机，或通过有线串口、蓝牙串口、UDP连接esp32下位机控制电机
- 配套esp32下位机：https://github.com/umas2022/MultiAxisHandler_ST3215
- 飞特官方上位机：https://gitee.com/ftservo/fddebug

```bash
pip install -r requirements.txt
pip install pyserial numpy pyqt5
```

## 项目结构

- MultiAxisSystem/
  - 多轴系统主函数，参见示例ControllerOne.py

- STServo_sdk/
  - st3215电机驱动sdk

- STServo_Python/
  - sdk附带的example，未来会删掉

- test/
  - 硬件性能测试

- toolbox/
  - 调试工具包


## 待办
- 串口和udp返回消息没有校验
- get_all_speed函数只完成了usb实现，serial和udp还没写
- 下位机串口消息最大64字节，超过会丢包