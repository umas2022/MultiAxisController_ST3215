"""
4足机器人，4自由度
规定四足向前摆动为正向
"""

import time
import sys

sys.path.append("..")
from MultiAxisSystem.MultiAxisController import MotorConfig
from MultiAxisSystem.MultiAxisControllerSerial import MultiAxisSerial
from MultiAxisSystem.MultiAxisControllerUSB import MultiAxisUSB
from MultiAxisSystem.MultiAxisControllerUDP import MultiAxisUdp


class ControllerDog4F:
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
            # left front
            MotorConfig(id=1, min=0, max=4096, init=2048, reverse=False),
            # right front
            MotorConfig(id=2, min=0, max=4096, init=2048, reverse=True),
            # left back
            MotorConfig(id=3, min=0, max=4096, init=2048, reverse=False),
            # right back
            MotorConfig(id=4, min=0, max=4096, init=2048, reverse=True),
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

    def go_forward(self):
        spd_list = [2000] * 4
        acc_list = [50] * 4
        period = 0.3

        self.ctrl.move_all_offset([300, 0, 0, 300], spd_list, acc_list)
        time.sleep(period)
        self.ctrl.move_all_offset([300, -300, -300, 300], spd_list, acc_list)
        time.sleep(period)
        self.ctrl.move_all_offset([0, -300, -300, 0], spd_list, acc_list)
        time.sleep(period)
        self.ctrl.move_all_offset([0, 0, 0, 0], spd_list, acc_list)
        time.sleep(period)

        self.ctrl.move_all_offset([0, 300, 300, 0], spd_list, acc_list)
        time.sleep(period)
        self.ctrl.move_all_offset([-300, 300, 300, -300], spd_list, acc_list)
        time.sleep(period)
        self.ctrl.move_all_offset([-300, 0, 0, -300], spd_list, acc_list)
        time.sleep(period)
        self.ctrl.move_all_offset([0, 0, 0, 0], spd_list, acc_list)

    def go_left(self):
        spd_list = [2000] * 4
        acc_list = [50] * 4
        period = 0.3

        self.ctrl.move_all_offset([-300, 0, 0, 300], spd_list, acc_list)
        time.sleep(period)
        self.ctrl.move_all_offset([-300, -300, 300, 300], spd_list, acc_list)
        time.sleep(period)
        self.ctrl.move_all_offset([0, -300, 300, 0], spd_list, acc_list)
        time.sleep(period)
        self.ctrl.move_all_offset([0, 0, 0, 0], spd_list, acc_list)
        time.sleep(period)

    def go_right(self):
        spd_list = [2000] * 4
        acc_list = [50] * 4
        period = 0.3

        self.ctrl.move_all_offset([-0, -300, 300, 0], spd_list, acc_list)
        time.sleep(period)
        self.ctrl.move_all_offset([-300, -300, 300, 300], spd_list, acc_list)
        time.sleep(period)
        self.ctrl.move_all_offset([-300, 0, 0, 300], spd_list, acc_list)
        time.sleep(period)
        self.ctrl.move_all_offset([0, 0, 0, 0], spd_list, acc_list)
        time.sleep(period)


if __name__ == "__main__":
    # Example usage
    dog4f = ControllerDog4F(mode="usb", serial_port="COM28")
    # dog4f = ControllerDog4F(mode="serial", serial_port="COM16")
    # dog4f = ControllerDog4F(mode="udp")
    dog4f.hardware_init()

    print("online_check")
    while not dog4f.online_check():
        time.sleep(1)

    print("move_all_init")
    dog4f.move_all_init(1000, 50)
    time.sleep(2)

    print("move test")
    while True:
        dog4f.go_forward()
        # dog4f.go_left()
        # dog4f.go_right()
