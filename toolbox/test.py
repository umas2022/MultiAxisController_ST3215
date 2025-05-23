import serial

# 替换为你的蓝牙虚拟串口号
PORT = "COM16"  # Windows 示例
# PORT = '/dev/rfcomm0'  # Linux 示例
# PORT = '/dev/cu.ESP32_Classic_BT-SerialPort'  # macOS 示例

baudrate = 115200  # 需与 ESP32 代码中的 Serial.begin() 一致

try:
    bt = serial.Serial(PORT, baudrate=baudrate, timeout=1)
    print("蓝牙串口连接成功！")

    while True:
        # 发送数据到 ESP32
        msg = input("输入要发送的消息: ")
        bt.write(msg.encode("utf-8"))

        # 接收 ESP32 返回的数据
        if bt.in_waiting:
            response = bt.readline().decode("utf-8").strip()
            print(f"收到回复: {response}")

except Exception as e:
    print(f"错误: {e}")
finally:
    if "bt" in locals():
        bt.close()
