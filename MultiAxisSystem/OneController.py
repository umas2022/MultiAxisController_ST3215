import sys

sys.path.append("..")
import time

from MultiAxisSystem.MultiAxisController import MultiAxisUSB, MultiAxisSerial, MotorConfig


class OneController:
    def __init__(self, mode="usb", serial_port=None):
        if mode == "usb":
            self.ctrl = MultiAxisUSB(serial_port)
        elif mode == "serial":
            self.ctrl = MultiAxisSerial(serial_port)
        else:
            raise ValueError("Unknown mode")

        self.ctrl.motors_list = [
            MotorConfig(id=1, min=0, max=4096, init=2048, reverse=False),
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


agent = OneController(mode="serial", serial_port="COM16")
agent.hardware_init()

print("online_check")
agent.ctrl.online_check()

print("move_all_init")
agent.move_all_init(1000, 50)

time.sleep(1)

print("move test")
agent.move_all_offset([2048], [1000], [50])

time.sleep(1)

print("get_all_position")
agent.get_all_position()
