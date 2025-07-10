"""
单轴示例，在self.ctrl.motors_list中添加电机配置
"""

import time
import sys

sys.path.append("..")

from MultiAxisSystem.MultiAxisController import MotorConfig
from MultiAxisSystem.MultiAxisControllerSerial import MultiAxisSerial
from MultiAxisSystem.MultiAxisControllerUSB import MultiAxisUSB
from MultiAxisSystem.MultiAxisControllerUDP import MultiAxisUdp


class ControllerOne:
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
            MotorConfig(id=7, min=0, max=4096, init=2048, reverse=False),
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
    # 示例用法
    agent = ControllerOne(mode="serial", serial_port="COM15")
    # agent = ControllerOne(mode="usb", serial_port="COM28")
    # # agent = ControllerOne(mode="udp")
    agent.hardware_init()

    print("online_check")
    agent.ctrl.online_check()

    print("move_all_init")
    agent.move_all_init(1000, 50)
    time.sleep(2)

    print("move test")
    agent.move_all_offset([1024], [1000], [50])
    time.sleep(2)

    # print("get_all_position")
    # print(agent.get_all_position())

    # print("get_all_load")
    # print(agent.get_all_load())

    # print("get_all_temper")
    # print(agent.get_all_temper())
