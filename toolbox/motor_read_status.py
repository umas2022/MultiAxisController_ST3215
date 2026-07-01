#!/usr/bin/env python3
"""Read position, speed, load, and temperature from multiple ST3215 motors."""

import time
from types import SimpleNamespace

from motor_toolbox_common import create_motor_driver
from multiaxis_driver.motor import MotorConfig
from multiaxis_driver.motor.STservo_sdk.stservo_def import MAX_ID


# ======================== 可调参数 ========================
CONNECTION_MODE = "usb"  # usb / serial / udp
COM_PORT = "COM5"
SERIAL_BAUDRATE = 115200
UDP_IP = "192.168.4.1"
UDP_PORT = 4210
LOCAL_IP = "0.0.0.0"
LOCAL_PORT = 4210

MOTOR_IDS = [1]
CONTINUOUS_READ = True
READ_INTERVAL = 0.2
# ==========================================================


def read_status(driver) -> bool:
    positions = driver.get_all_position()
    speeds = driver.get_all_speed()
    loads = driver.get_all_load()
    temperatures = driver.get_all_temper()
    expected = len(MOTOR_IDS)
    if any(len(values) != expected for values in (positions, speeds, loads, temperatures)):
        print("[ERROR] Failed to read a complete motor status response.")
        return False

    for index, motor_id in enumerate(MOTOR_IDS):
        print(
            f"ID {motor_id:3d} | position={positions[index]:4d} | "
            f"speed={speeds[index]:6d} | load={loads[index]:6d} | "
            f"temperature={temperatures[index]:3d} C"
        )
    print("-" * 72)
    return True


def main() -> int:
    if not MOTOR_IDS or len(set(MOTOR_IDS)) != len(MOTOR_IDS):
        raise ValueError("MOTOR_IDS must contain unique motor IDs")
    if any(not 0 <= motor_id <= MAX_ID for motor_id in MOTOR_IDS):
        raise ValueError(f"Motor IDs must be in range 0-{MAX_ID}")

    connection = SimpleNamespace(
        mode=CONNECTION_MODE, port=COM_PORT, serial_baudrate=SERIAL_BAUDRATE,
        udp_ip=UDP_IP, udp_port=UDP_PORT, local_ip=LOCAL_IP, local_port=LOCAL_PORT,
    )
    driver = create_motor_driver(connection)
    driver.motors_list = [
        MotorConfig(id=motor_id, min=0, max=4095, init=2048, reverse=False)
        for motor_id in MOTOR_IDS
    ]

    try:
        while True:
            if not read_status(driver):
                return 1
            if not CONTINUOUS_READ:
                return 0
            time.sleep(READ_INTERVAL)
    except KeyboardInterrupt:
        print("\n[INFO] Stopped.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
