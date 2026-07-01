# multiaxis-driver

与具体机器人无关的硬件驱动包，包含：

- ST3215 电机 USB、串口转发和 UDP 驱动
- IMU 驱动
- ELRS/CRSF 接收机驱动

开发模式安装：

```powershell
pip install -e .\drivers
```

机器人控制代码只能依赖本包；本包不能依赖 `robot`。
