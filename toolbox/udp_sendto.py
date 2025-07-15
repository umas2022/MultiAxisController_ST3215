"""
udp消息发送（到esp32）
"""

import socket

# ESP32 的默认 IP 是 192.168.4.1（SoftAP 模式下）
esp32_ip = "192.168.4.1"
esp32_port = 4210

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


# sock.sendto(b"hi", ("192.168.4.1", 4210))
# sock.sendto(bytes([0x01, 0x02, 0x03, 0x04, 0x05]), (esp32_ip, esp32_port))
sock.sendto(bytes([0xAA, 0x01, 0x01, 0x01, 0x03]), (esp32_ip, esp32_port))
