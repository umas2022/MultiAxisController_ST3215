
'''
4足机器人，4自由度
规定四足向前摆动为正向
'''

import time
import sys
sys.path.append("..")
from MultiAxisSystem.MultiAxisController import MultiAxisUSB, MultiAxisSerial, MotorConfig


class ControllerDog4F:
    def __init__(self, mode="usb", serial_port=None):
        if mode == "usb":
            self.ctrl = MultiAxisUSB(serial_port)
        elif mode == "serial":
            self.ctrl = MultiAxisSerial(serial_port)
        else:
            raise ValueError("Unknown mode")

        self.ctrl.motors_list = [
            MotorConfig(id=1, min=0, max=4096, init=2048, reverse=False),
            MotorConfig(id=2, min=0, max=4096, init=2048, reverse=True),
            MotorConfig(id=3, min=0, max=4096, init=2048, reverse=False),
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



dog4f = ControllerDog4F(mode="serial", serial_port="COM21")
dog4f.hardware_init()

print("online_check")
dog4f.ctrl.online_check()

print("move_all_init")
dog4f.move_all_init(1000, 50)
time.sleep(2)

print("move test")
while True:
    dog4f.move_all_offset([512]*4, [2000]*4, [50]*4)
    time.sleep(1)
    dog4f.move_all_offset([-512]*4, [2000]*4, [50]*4)
    time.sleep(1)
