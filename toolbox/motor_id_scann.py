#!/usr/bin/env python3
"""
Scan ST3215 / STS servo IDs over usb, ESP32 serial/Bluetooth serial, or UDP.
"""

from types import SimpleNamespace
from typing import List

from motor_toolbox_common import create_motor_driver
from multiaxis_driver.motor.STservo_sdk.stservo_def import MAX_ID


# ======================== 可调参数 ========================
# usb: 电脑通过 USB 转串口直接连接舵机
# serial: 电脑通过串口/蓝牙串口连接 ESP32
# udp: 电脑通过 Wi-Fi UDP 连接 ESP32
CONNECTION_MODE = "usb"

# usb 和 serial 模式使用的 COM 口
COM_PORT = "COM16"

# serial 模式参数
SERIAL_BAUDRATE = 115200

# udp 模式参数
UDP_IP = "192.168.4.1"
UDP_PORT = 4210
LOCAL_IP = "0.0.0.0"
LOCAL_PORT = 4210

# 要扫描的舵机 ID 范围（包含首尾）
START_ID = 1
END_ID = 20
# ==========================================================


def scan_motors(driver, start_id: int = 1, end_id: int = 20) -> List[int]:
    if not (0 <= start_id <= MAX_ID and 0 <= end_id <= MAX_ID):
        print(f"[ERROR] Invalid scan range {start_id}-{end_id}. Valid range is 0-{MAX_ID}.")
        return []
    if start_id > end_id:
        print(f"[ERROR] Invalid scan range {start_id}-{end_id}. start_id must be <= end_id.")
        return []

    online_motors: List[int] = []
    print(f"[INFO] Scanning motor IDs {start_id}-{end_id}")
    print("-" * 50)

    for motor_id in range(start_id, end_id + 1):
        if driver.ping_motor(motor_id):
            online_motors.append(motor_id)
            print(f"[OK] ID {motor_id:3d} online")
        else:
            print(f"[--] ID {motor_id:3d} offline")

    print("-" * 50)
    if online_motors:
        print(f"[OK] Found {len(online_motors)} online motor(s): {online_motors}")
    else:
        print("[WARN] No online motors found.")
    return online_motors


def main():
    connection = SimpleNamespace(
        mode=CONNECTION_MODE,
        port=COM_PORT,
        serial_baudrate=SERIAL_BAUDRATE,
        udp_ip=UDP_IP,
        udp_port=UDP_PORT,
        local_ip=LOCAL_IP,
        local_port=LOCAL_PORT,
    )
    driver = create_motor_driver(connection)
    online_motors = scan_motors(driver, start_id=START_ID, end_id=END_ID)
    raise SystemExit(0 if online_motors else 1)


if __name__ == "__main__":
    main()
