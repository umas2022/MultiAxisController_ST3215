#!/usr/bin/env python3
"""
Change the ID of an ST3215 / STS servo over usb, ESP32 serial, or UDP.
"""

from types import SimpleNamespace
from typing import List, Optional

from motor_toolbox_common import create_motor_driver
from multiaxis_driver.motor.STservo_sdk.stservo_def import MAX_ID


# ======================== 可调参数 ========================
CONNECTION_MODE = "usb"  # usb / serial / udp
COM_PORT = "COM16"        # usb、serial 模式使用
SERIAL_BAUDRATE = 115200
UDP_IP = "192.168.4.1"
UDP_PORT = 4210
LOCAL_IP = "0.0.0.0"
LOCAL_PORT = 4210

OLD_ID = 5  # 当前 ID；None 表示自动查找唯一在线舵机
NEW_ID = 1     # 要设置的新 ID
SCAN_START = 1
SCAN_END = 20
# ==========================================================


def scan_online_motors(driver, start_id: int, end_id: int) -> List[int]:
    return [current_id for current_id in range(start_id, end_id + 1) if driver.ping_motor(current_id)]


def resolve_old_id(driver, old_id: Optional[int], scan_start: int, scan_end: int) -> Optional[int]:
    if old_id is not None:
        if not (0 <= old_id <= MAX_ID):
            print(f"[ERROR] Invalid old ID: {old_id}. Valid range is 0-{MAX_ID}.")
            return None
        if driver.ping_motor(old_id):
            return old_id
        print(f"[WARN] Motor with old ID {old_id} did not respond.")

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

    if old_id is None and len(online_motors) == 1:
        detected_id = online_motors[0]
        print(f"[INFO] Auto-detected current motor ID: {detected_id}")
        return detected_id

    print(f"[INFO] Online motor IDs: {online_motors}")
    if old_id is not None:
        print("[ERROR] The requested old ID is not online.")
    else:
        print("[ERROR] Multiple motors are online. Set OLD_ID in the script.")
    return None


def change_motor_id(driver, old_id: Optional[int], new_id: int, scan_start: int = 1, scan_end: int = 20) -> bool:
    if not (0 <= new_id <= MAX_ID):
        print(f"[ERROR] Invalid new ID: {new_id}. Valid range is 0-{MAX_ID}.")
        return False

    current_id = resolve_old_id(driver, old_id, scan_start, scan_end)
    if current_id is None:
        return False

    if current_id == new_id:
        print("[ERROR] Old ID and new ID are identical. Nothing to change.")
        return False

    if driver.ping_motor(new_id):
        print(f"[ERROR] New ID {new_id} is already in use. Choose an unused ID.")
        return False

    if not driver.set_motor_id(current_id, new_id):
        print(f"[ERROR] Failed to change motor ID: {current_id} -> {new_id}")
        return False

    if driver.ping_motor(current_id):
        print(f"[WARN] Old ID {current_id} still responds. Check whether multiple motors are on the bus.")

    if not driver.ping_motor(new_id):
        print(f"[ERROR] ID change command was sent, but new ID {new_id} did not respond.")
        return False

    print(f"[OK] Motor ID changed successfully: {current_id} -> {new_id}")
    return True


def main():
    connection = SimpleNamespace(
        mode=CONNECTION_MODE, port=COM_PORT, serial_baudrate=SERIAL_BAUDRATE,
        udp_ip=UDP_IP, udp_port=UDP_PORT, local_ip=LOCAL_IP, local_port=LOCAL_PORT,
    )
    driver = create_motor_driver(connection)
    success = change_motor_id(driver, old_id=OLD_ID, new_id=NEW_ID, scan_start=SCAN_START, scan_end=SCAN_END)
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
