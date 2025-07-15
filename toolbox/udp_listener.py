"""
udp消息监听（自esp32）
"""

import socket

esp32_ip = "192.168.4.1"
esp32_port = 4210
local_port = 4210  # 本地端口必须手动指定

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", local_port))  # 绑定固定端口用于发送和接收
sock.sendto(b"hi", (esp32_ip, esp32_port))

print(f"Sent to {esp32_ip}:{esp32_port} from local port {local_port}")

# 接收回传
while True:
    data, addr = sock.recvfrom(1024)
    print(f"Received from {addr}: {data}")
