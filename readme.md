# st3215多轴运动系统python上位机

- 支持上位机usb直连电机，或通过有线串口、蓝牙串口、UDP连接esp32下位机控制电机
- 配套esp32下位机：https://github.com/umas2022/MultiAxisHandler_ST3215
- 飞特官方上位机：https://gitee.com/ftservo/fddebug

```bash
# conda activate multiaxiscontroller
pip install -r requirements.txt
pip install pyserial numpy 
```

## 项目结构

- scripts/
  - 项目入口

- src/drivers
  - 驱动

- src/controllers
  - 多轴系统控制类，参见示例ControllerOne.py

- toolbox/
  - 调试工具


## todo
- 串口和udp返回消息没有校验
- get_all_speed函数只完成了usb实现，serial和udp还没写
- 下位机串口消息最大64字节，超过会丢包