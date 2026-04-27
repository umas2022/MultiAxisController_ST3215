#!/usr/bin/env python3
"""
Read IMU data over direct USB or ESP32 Bluetooth serial forwarding.
"""

import argparse
import os
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROBOT_ROOT = os.path.join(PROJECT_ROOT, "robot")
for extra_path in (PROJECT_ROOT, ROBOT_ROOT):
    if extra_path not in sys.path:
        sys.path.append(extra_path)

from robot.src.drivers.imu_driver import IMUDriverSerial, IMUDriverUSB


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read IMU data from USB or ESP32 serial forwarding.")
    parser.add_argument("--mode", choices=("usb", "serial"), default="usb")
    parser.add_argument("--port", default=None, help='Port like "COM9" for usb or "COM8" for serial bridge.')
    parser.add_argument("--baudrate", type=int, default=None, help="Override baudrate. usb default 9600, serial default 115200.")
    parser.add_argument("--samples", type=int, default=5, help="Number of samples to print. Default: 5")
    parser.add_argument("--interval", type=float, default=0.3, help="Seconds between samples. Default: 0.3")
    return parser.parse_args()


def create_driver(args):
    if args.mode == "usb":
        port = args.port or "COM9"
        baudrate = args.baudrate or 9600
        return IMUDriverUSB(port=port, baud=baudrate)
    port = args.port or "COM8"
    baudrate = args.baudrate or 115200
    return IMUDriverSerial(port=port, baud=baudrate)


def main():
    args = parse_args()
    driver = create_driver(args)
    if not driver.hardware_init():
        raise SystemExit(1)

    ready = driver.wait_for_data(2.0)
    print(f"[INFO] wait_for_data={ready}")

    for index in range(args.samples):
        if args.mode == "serial":
            driver.refresh()
        acc, gyro, angle = driver.get_data()
        print(f"[{index}] acc={acc} gyro={gyro} angle={angle}")
        time.sleep(args.interval)

    driver.close()


if __name__ == "__main__":
    main()
