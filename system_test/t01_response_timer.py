"""
响应速度测试
pc端，win10，1~6号电机全部在线时，遍历6个电机耗时约8ms
树莓派4b，4G内存，ubuntu22 server，遍历6个电机耗时约12ms
"""

import sys
import time

sys.path.append("..")
from STservo_sdk import *  # Uses STServo SDK library

# Default setting
STS_ID = 1  # STServo ID : 1
BAUDRATE = 1000000  # STServo default baudrate : 1000000
DEVICENAME = "COM4"  # Check which port is being used on your controller
# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = sts(portHandler)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

while 1:

    time_start = time.perf_counter()

    for servo_id in range(6):

        sts_present_position, sts_comm_result, sts_error = packetHandler.ReadPos(servo_id + 1)
        if sts_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(sts_comm_result))
        else:
            print("[ID:%03d] PresPos:%d" % (servo_id + 1, sts_present_position))
        if sts_error != 0:
            print(packetHandler.getRxPacketError(sts_error))

    time_cost = (time.perf_counter() - time_start) * 1000
    print("Time cost: %.2f ms" % time_cost)

    time.sleep(1)

# Close port
portHandler.closePort()
