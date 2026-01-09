"""
udp消息发送
"""

import socket
import time
import struct
import sys

sys.path.append("..")
from robot.src.controller.ControllerArm6F import ControllerArm

target_ip = "172.23.64.52"
target_port = 8888
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# 初始化控制器
arm_leader = ControllerArm(mode="usb", serial_port="COM31")

arm_leader.hardware_init()


# 在线检查
while True:
    print("leader arm online check ...")
    if arm_leader.online_check():
        break
    time.sleep(1)


print("Sending joint positions...")

while True:
    try:
        # '!' 表示网络字节序，6个 int32
        packed_data = struct.pack("!6i", *arm_leader.get_all_position())
        sock.sendto(packed_data, (target_ip, target_port))
        time.sleep(0.1)
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(1)
