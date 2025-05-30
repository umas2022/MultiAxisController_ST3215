"""
4足机器人，8自由度，连杆式
规定四足向前摆动为正向
"""

import time
import sys

sys.path.append("..")
from MultiAxisSystem.MultiAxisController import MultiAxisUSB, MultiAxisSerial, MultiAxisUdp, MotorConfig


class ControllerDog8F:
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
            # left front shank
            MotorConfig(id=1, min=0, max=4096, init=2048, reverse=False),
            # left front thigh
            MotorConfig(id=2, min=0, max=4096, init=2048, reverse=False),
            # right front shank
            MotorConfig(id=3, min=0, max=4096, init=2048, reverse=True),
            # right front thigh
            MotorConfig(id=4, min=0, max=4096, init=2048, reverse=True),
            # left back shank
            MotorConfig(id=5, min=0, max=4096, init=2048, reverse=False),
            # left back thigh
            MotorConfig(id=6, min=0, max=4096, init=2048, reverse=False),
            # right back shank
            MotorConfig(id=7, min=0, max=4096, init=2048, reverse=True),
            # right back thigh
            MotorConfig(id=8, min=0, max=4096, init=2048, reverse=True),
        ]

    def hardware_init(self) -> bool:
        return self.ctrl.hardware_init()

    def online_check(self) -> bool:
        return self.ctrl.online_check()

    def move_all_init(self, spd, acc) -> None:
        self.ctrl.move_all_init(spd, acc)

    def stand_up(self):
        self.ctrl.move_all_offset([-500, -300, -500, -300, -500, -300, -500, -300], [1000] * 8, [50] * 8)

    def go_forward(self):
        spd_list = [2000] * 8
        acc_list = [50] * 8
        period = 0.25

        # 后落地
        self.ctrl.move_all_offset([-700, -600, -100, 0, -100, 0, -700, -600], spd_list, acc_list)
        time.sleep(period)
        self.ctrl.move_all_offset([-700, -600, -100, 0, -100, 0, -700, -600], spd_list, acc_list)
        time.sleep(period)
        # 后抬腿
        self.ctrl.move_all_offset([-500, -600, -500, 0, -500, 0, -500, -600], spd_list, acc_list)
        time.sleep(period)
        # 前抬腿
        self.ctrl.move_all_offset([-100, 0, -700, -600, -700, -600, -100, 0], spd_list, acc_list)
        time.sleep(period)
        self.ctrl.move_all_offset([-100, 0, -700, -600, -700, -600, -100, 0], spd_list, acc_list)
        time.sleep(period)
        # 前落地
        self.ctrl.move_all_offset([-500, 0, -500, -600, -500, -600, -100, 0], spd_list, acc_list)
        time.sleep(period)

    def go_left(self):
        pass

    def go_right(self):
        pass


dog8f = ControllerDog8F(mode="serial", serial_port="COM15")
# dog8f = ControllerDog8F(mode="udp")
dog8f.hardware_init()

print("online_check")
dog8f.ctrl.online_check()

print("move_all_init")
dog8f.move_all_init(1000, 50)
time.sleep(2)

print("stand up")
dog8f.stand_up()
time.sleep(2)

print("move test")
while True:
    dog8f.go_forward()
    # dog8f.go_left()
    # dog8f.go_right()
