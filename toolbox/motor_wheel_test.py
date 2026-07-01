#!/usr/bin/env python3
"""Switch one ST3215 motor to wheel mode and run a safe speed sequence."""

import time
from types import SimpleNamespace

from motor_toolbox_common import create_motor_driver
from multiaxis_driver.motor.STservo_sdk.stservo_def import MAX_ID


# ======================== 可调参数 ========================
CONNECTION_MODE = "usb"  # usb / serial / udp
COM_PORT = "COM5"
SERIAL_BAUDRATE = 115200
UDP_IP = "192.168.4.1"
UDP_PORT = 4210
LOCAL_IP = "0.0.0.0"
LOCAL_PORT = 4210

MOTOR_ID = 1
SPEED_SEQUENCE = [800, 0, -800, 0]
ACCELERATION = 50
STEP_DURATION = 1.0

# 轮式模式会让电机连续旋转。确认电机已架空后改为 True。
ENABLE_WHEEL_TEST = False
# ==========================================================


def main() -> int:
    if not 0 <= MOTOR_ID <= MAX_ID:
        raise ValueError(f"MOTOR_ID must be in range 0-{MAX_ID}")
    if not 0 <= ACCELERATION <= 254:
        raise ValueError("ACCELERATION must be in range 0-254")
    if not ENABLE_WHEEL_TEST:
        print("[SAFETY] Wheel test is disabled. Raise the motor off the ground and set ENABLE_WHEEL_TEST = True.")
        return 1

    connection = SimpleNamespace(
        mode=CONNECTION_MODE, port=COM_PORT, serial_baudrate=SERIAL_BAUDRATE,
        udp_ip=UDP_IP, udp_port=UDP_PORT, local_ip=LOCAL_IP, local_port=LOCAL_PORT,
    )
    driver = create_motor_driver(connection)
    if not driver.ping_motor(MOTOR_ID):
        print(f"[ERROR] Motor ID {MOTOR_ID} is offline.")
        return 1
    if not driver.set_wheel_mode(MOTOR_ID):
        print(f"[ERROR] Failed to switch motor ID {MOTOR_ID} to wheel mode.")
        return 1

    print(f"[OK] Motor ID {MOTOR_ID} entered wheel mode.")
    try:
        for speed in SPEED_SEQUENCE:
            if not driver.set_motor_speed(MOTOR_ID, speed, ACCELERATION):
                print(f"[ERROR] Failed to set speed {speed}.")
                return 1
            print(f"[INFO] speed={speed}, acceleration={ACCELERATION}")
            time.sleep(STEP_DURATION)
    finally:
        if not driver.set_motor_speed(MOTOR_ID, 0, ACCELERATION):
            print("[WARN] Failed to send the final stop command.")
        else:
            print("[OK] Motor stopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
