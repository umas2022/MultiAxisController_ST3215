#!/usr/bin/env python3
"""
Read the current position of an ST3215 / STS servo over usb, ESP32 serial, or UDP.
"""

from types import SimpleNamespace
from typing import List, Optional

from motor_toolbox_common import create_motor_driver
from robot.src.drivers.motor_driver.STservo_sdk import MAX_ID


# ======================== 可调参数 ========================
CONNECTION_MODE = "usb"  # usb / serial / udp
COM_PORT = "COM5"        # usb、serial 模式使用
SERIAL_BAUDRATE = 115200
UDP_IP = "192.168.4.1"
UDP_PORT = 4210
LOCAL_IP = "0.0.0.0"
LOCAL_PORT = 4210

MOTOR_ID = None  # 指定舵机 ID；None 表示自动查找唯一在线舵机
SCAN_START = 1
SCAN_END = 20
# ==========================================================


def scan_online_motors(driver, start_id: int = 1, end_id: int = 20) -> List[int]:
    return [current_id for current_id in range(start_id, end_id + 1) if driver.ping_motor(current_id)]


def resolve_target_id(driver, requested_id: Optional[int], scan_start: int, scan_end: int) -> Optional[int]:
    if requested_id is not None:
        if not (0 <= requested_id <= MAX_ID):
            print(f"[ERROR] Invalid motor ID: {requested_id}. Valid range is 0-{MAX_ID}.")
            return None
        if driver.ping_motor(requested_id):
            return requested_id
        print(f"[WARN] Motor ID {requested_id} did not respond.")

    if not (0 <= scan_start <= scan_end <= MAX_ID):
        print(f"[ERROR] Invalid scan range {scan_start}-{scan_end}. Valid range is 0-{MAX_ID}.")
        return None

    online_motors = scan_online_motors(driver, scan_start, scan_end)
    if not online_motors:
        print(
            f"[ERROR] No online motors found in ID range {scan_start}-{scan_end}. "
            "Check power, wiring, connection mode, and whether another program is occupying the link."
        )
        return None

    if requested_id is None and len(online_motors) == 1:
        detected_id = online_motors[0]
        print(f"[INFO] Auto-detected motor ID: {detected_id}")
        return detected_id

    print(f"[INFO] Online motor IDs: {online_motors}")
    if requested_id is not None:
        print("[ERROR] The requested motor ID is not online.")
    else:
        print("[ERROR] Multiple motors are online. Set MOTOR_ID in the script.")
    return None


def read_motor_position(driver, motor_id: Optional[int], scan_start: int = 1, scan_end: int = 20) -> Optional[int]:
    target_id = resolve_target_id(driver, motor_id, scan_start, scan_end)
    if target_id is None:
        return None

    position = driver.get_motor_position(target_id)
    if position is None:
        print(f"[ERROR] Failed to read position from motor ID {target_id}.")
        return None

    print(f"Motor ID {target_id} current raw position: {position}")
    return position


def main():
    connection = SimpleNamespace(
        mode=CONNECTION_MODE, port=COM_PORT, serial_baudrate=SERIAL_BAUDRATE,
        udp_ip=UDP_IP, udp_port=UDP_PORT, local_ip=LOCAL_IP, local_port=LOCAL_PORT,
    )
    driver = create_motor_driver(connection)
    position = read_motor_position(driver, motor_id=MOTOR_ID, scan_start=SCAN_START, scan_end=SCAN_END)
    raise SystemExit(0 if position is not None else 1)


if __name__ == "__main__":
    main()
