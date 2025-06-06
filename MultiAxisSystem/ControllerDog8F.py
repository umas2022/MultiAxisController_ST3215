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
        self.ctrl.move_all_offset([-600, -400, -600, -400, -600, -400, -600, -400], [1000] * 8, [50] * 8)

    def go_forward(self):
        """
        电机1:大腿; 电机2:小腿
        平行四边形结构约束,电机1与电机2转角一致时,腿部连杆关节角度不变,此时8dof可视为4dof
        调整后摆稍稍降低, 消除4dof原点摩擦: 中点[-600,-400]，前摆[-400, -200]，后摆[-800, -650]
        """
        spd_list = [2000] * 8
        acc_list = [50] * 8
        period = 0.2

        # G1前G2中
        self.ctrl.move_all_offset([-400, -200, -600, -400, -600, -400, -400, -200], spd_list, acc_list)
        time.sleep(period)
        # G1前G2后
        self.ctrl.move_all_offset([-400, -200, -800, -650, -800, -650, -400, -200], spd_list, acc_list)
        time.sleep(period)
        # G1中G2后
        self.ctrl.move_all_offset([-600, -400, -800, -650, -800, -650, -600, -400], spd_list, acc_list)
        time.sleep(period)
        # G1中G2中
        self.ctrl.move_all_offset([-600, -400, -600, -400, -600, -400, -600, -400], spd_list, acc_list)
        time.sleep(period)

        # G2前G1中
        self.ctrl.move_all_offset([-600, -400, -400, -200, -400, -200, -600, -400], spd_list, acc_list)
        time.sleep(period)
        # G2前G1后
        self.ctrl.move_all_offset([-800, -650, -400, -200, -400, -200, -800, -650], spd_list, acc_list)
        time.sleep(period)
        # G2中G1后
        self.ctrl.move_all_offset([-800, -650, -600, -400, -600, -400, -800, -650], spd_list, acc_list)
        time.sleep(period)
        # G1中G2中
        self.ctrl.move_all_offset([-600, -400, -600, -400, -600, -400, -600, -400], spd_list, acc_list)
        time.sleep(period)

    def go_left(self):
        pass

    def go_right(self):
        pass


dog8f = ControllerDog8F(mode="serial", serial_port="COM25")
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
