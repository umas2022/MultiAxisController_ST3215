"""
elrs接收机16通道串口数据解析
"""

import serial
import time

# CRSF 协议常量
CRSF_ADDRESS = 0xC8
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
EXPECTED_PAYLOAD_LENGTH = 22  # RC数据包的 payload 长度
TOTAL_PACKET_LENGTH = EXPECTED_PAYLOAD_LENGTH + 4  # addr + len + type + payload + crc

# 串口初始化
ser = serial.Serial("COM3", 420000, timeout=0.1)

buffer = bytearray()
last_print_time = time.time()


def parse_crsf_packet(packet):
    if len(packet) < 5:  # addr + len + type + 1字节payload + crc
        return None

    addr = packet[0]
    length = packet[1]
    if length + 2 != len(packet):  # 帧长度 = length + addr/len 字节
        return None

    type_ = packet[2]
    if addr == CRSF_ADDRESS and type_ == CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
        payload = packet[3:-1]  # 去掉crc
        if len(payload) != EXPECTED_PAYLOAD_LENGTH:
            return None

        # 解包22字节的RC通道数据
        bits = int.from_bytes(payload, byteorder="little")
        channels = []
        for i in range(16):
            value = (bits >> (i * 11)) & 0x7FF
            channels.append(value)
        return channels
    return None


try:
    while True:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            buffer.extend(data)

            # 尝试从 buffer 中提取合法的帧
            while len(buffer) >= 3:
                if buffer[0] != CRSF_ADDRESS:
                    buffer.pop(0)
                    continue

                frame_len = buffer[1]
                total_len = frame_len + 2  # addr + len + payload + crc
                if len(buffer) < total_len:
                    break  # 等待更多数据

                packet = buffer[:total_len]
                buffer = buffer[total_len:]

                result = parse_crsf_packet(packet)
                if result:
                    now = time.time()
                    if now - last_print_time >= 1.0:
                        print("Channels:", result)
                        last_print_time = now
        else:
            time.sleep(0.01)

except KeyboardInterrupt:
    print("用户中断，退出程序")
finally:
    ser.close()
