"""
realman遥操臂
8dof
"""

import time

from src.drivers.motor_driver.MultiAxisController import MotorConfig
from src.drivers.motor_driver.MultiAxisControllerSerial import MultiAxisSerial
from src.drivers.motor_driver.MultiAxisControllerUSB import MultiAxisUSB
from src.drivers.motor_driver.MultiAxisControllerUDP import MultiAxisUdp


class ControllerArm:
    def __init__(self, mode="usb", serial_port=None):
        if mode == "usb":
            self.ctrl = MultiAxisUSB(serial_port)
        elif mode == "serial":
            self.ctrl = MultiAxisSerial(serial_port)
        elif mode == "udp":
            self.ctrl = MultiAxisUdp()
        else:
            raise ValueError("Unknown mode")

        self.ctrl.motors_list = [
            MotorConfig(id=1, min=0, max=4096, init=2048, reverse=False),
            MotorConfig(id=2, min=0, max=4096, init=2048, reverse=False),
            MotorConfig(id=3, min=0, max=4096, init=2048, reverse=False),
            MotorConfig(id=4, min=0, max=4096, init=2048, reverse=False),
            MotorConfig(id=5, min=0, max=4096, init=2048, reverse=False),
            MotorConfig(id=6, min=0, max=4096, init=2048, reverse=False),
            MotorConfig(id=7, min=0, max=4096, init=2048, reverse=False),
            MotorConfig(id=8, min=0, max=4096, init=2048, reverse=False),
        ]

    def hardware_init(self) -> bool:
        return self.ctrl.hardware_init()

    def online_check(self) -> bool:
        return self.ctrl.online_check()

    def move_all_init(self, spd, acc) -> None:
        self.ctrl.move_all_init(spd, acc)

    def move_all_absolute(self, pos, spd, acc) -> None:
        self.ctrl.move_all_absolute(pos, spd, acc)

    def move_all_offset(self, offset, spd, acc) -> None:
        self.ctrl.move_all_offset(offset, spd, acc)

    def get_all_position(self) -> list:
        return self.ctrl.get_all_position()

    def get_all_load(self) -> list:
        return self.ctrl.get_all_load()

    def get_all_temper(self) -> list:
        return self.ctrl.get_all_temper()


if __name__ == "__main__":
    # Example usage
    arm = ControllerArm(mode="usb", serial_port="COM7")
    # arm = ControllerArm(mode="usb", serial_port="/dev/ttyACM0")
    arm.hardware_init()

    print("online_check")
    while not arm.online_check():
        time.sleep(1)

    while True:
        print(arm.get_all_position())
        time.sleep(1)
