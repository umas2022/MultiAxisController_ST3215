#!/usr/bin/env python3
"""Move one or more ST3215 motors to absolute positions."""

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

# 多电机列表长度必须一致。注册写入完成后统一执行，电机会同步开始运动。
MOTOR_IDS = [1]
TARGET_POSITIONS = [2048]
SPEEDS = [500]
ACCELERATIONS = [50]

WAIT_UNTIL_REACHED = True
POSITION_TOLERANCE = 20
WAIT_TIMEOUT = 10.0
POLL_INTERVAL = 0.05
# ==========================================================


def validate_settings() -> None:
    lengths = {len(MOTOR_IDS), len(TARGET_POSITIONS), len(SPEEDS), len(ACCELERATIONS)}
    if lengths != {len(MOTOR_IDS)} or not MOTOR_IDS:
        raise ValueError("Motor ID, position, speed, and acceleration lists must have the same non-zero length")
    if len(set(MOTOR_IDS)) != len(MOTOR_IDS):
        raise ValueError("Motor IDs must be unique")
    if any(not 0 <= motor_id <= MAX_ID for motor_id in MOTOR_IDS):
        raise ValueError(f"Motor IDs must be in range 0-{MAX_ID}")
    if any(not 0 <= position <= 4095 for position in TARGET_POSITIONS):
        raise ValueError("Target positions must be in range 0-4095")
    if any(speed < 0 for speed in SPEEDS):
        raise ValueError("Position-mode speeds must be non-negative")
    if any(not 0 <= acc <= 254 for acc in ACCELERATIONS):
        raise ValueError("Accelerations must be in range 0-254")


def wait_until_reached(driver) -> bool:
    deadline = time.monotonic() + WAIT_TIMEOUT
    while time.monotonic() < deadline:
        positions = driver.get_all_position()
        if len(positions) != len(TARGET_POSITIONS):
            print("[ERROR] Failed to read all motor positions.")
            return False

        errors = [abs(actual - target) for actual, target in zip(positions, TARGET_POSITIONS)]
        print(f"[INFO] positions={positions}, errors={errors}")
        if all(error <= POSITION_TOLERANCE for error in errors):
            return True
        time.sleep(POLL_INTERVAL)

    print(f"[ERROR] Motors did not reach their targets within {WAIT_TIMEOUT:.1f} seconds.")
    return False


def main() -> int:
    validate_settings()
    connection = SimpleNamespace(
        mode=CONNECTION_MODE, port=COM_PORT, serial_baudrate=SERIAL_BAUDRATE,
        udp_ip=UDP_IP, udp_port=UDP_PORT, local_ip=LOCAL_IP, local_port=LOCAL_PORT,
    )
    driver = create_motor_driver(connection)
    driver.motors_list = [
        MotorConfig(id=motor_id, min=0, max=4095, init=2048, reverse=False)
        for motor_id in MOTOR_IDS
    ]

    if not driver.online_check():
        print("[ERROR] One or more configured motors are offline.")
        return 1

    driver.move_all_absolute(TARGET_POSITIONS, SPEEDS, ACCELERATIONS)
    print(f"[OK] Move command sent: IDs={MOTOR_IDS}, targets={TARGET_POSITIONS}")

    if WAIT_UNTIL_REACHED and not wait_until_reached(driver):
        return 1
    print("[OK] Motion completed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
