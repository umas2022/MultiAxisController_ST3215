"""
4足机器人，12自由度，直连
规定由折叠平躺到站立方向为正
"""

import time
import sys

sys.path.append("..")

from MultiAxisSystem.MultiAxisController import MultiAxisUSB, MultiAxisSerial, MultiAxisUdp, MotorConfig


class ControllerDog12F:
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
            # LF
            MotorConfig(id=11, min=0, max=4096, init=2048, reverse=False),
            MotorConfig(id=12, min=0, max=4096, init=2048, reverse=True),
            MotorConfig(id=13, min=0, max=4096, init=2048, reverse=True),
            # RF
            MotorConfig(id=14, min=0, max=4096, init=2048, reverse=True),
            MotorConfig(id=15, min=0, max=4096, init=2048, reverse=False),
            MotorConfig(id=16, min=0, max=4096, init=2048, reverse=False),
            # LH
            MotorConfig(id=17, min=0, max=4096, init=2048, reverse=True),
            MotorConfig(id=18, min=0, max=4096, init=2048, reverse=False),
            MotorConfig(id=19, min=0, max=4096, init=2048, reverse=False),
            # RH
            MotorConfig(id=20, min=0, max=4096, init=2048, reverse=False),
            MotorConfig(id=21, min=0, max=4096, init=2048, reverse=True),
            MotorConfig(id=22, min=0, max=4096, init=2048, reverse=True),
        ]

        self.posture_stand = [0, -500, -1000] * 4  # 站立姿态

    def hardware_init(self) -> bool:
        return self.ctrl.hardware_init()

    def online_check(self) -> bool:
        return self.ctrl.online_check()

    def move_all_init(self, spd, acc) -> None:
        self.ctrl.move_all_init(spd, acc)

    def move_all_offset(self, offset, spd, acc) -> None:
        self.ctrl.move_all_offset(offset, spd, acc)

    def get_all_position(self) -> list:
        return self.ctrl.get_all_position()

    def get_all_load(self) -> list:
        return self.ctrl.get_all_load()

    def get_all_temper(self) -> list:
        return self.ctrl.get_all_temper()

    def stand_up(self):
        self.move_all_offset(
            self.posture_stand,
            [1000, 1000, 2000] * 12,
            [50, 50, 100] * 12,
        )

    def hw_go_forward(self):
        period = 0.5
        pl_front = [0, -800, -1100]
        pl_middle = [0, -500, -1000]
        # lf + rf + lh + rh
        self.move_all_offset(pl_middle + pl_middle + pl_middle + pl_middle, [1000] * 12, [100] * 12)
        time.sleep(period)
        self.move_all_offset(pl_front + pl_middle + pl_middle + pl_front, [1000] * 12, [100] * 12)
        time.sleep(period)
        self.move_all_offset(pl_front + pl_front + pl_front + pl_front, [1000] * 12, [100] * 12)
        time.sleep(period)
        self.move_all_offset(pl_middle + pl_front + pl_front + pl_middle, [1000] * 12, [100] * 12)
        time.sleep(period)


# test_agent = ControllerDog12F(mode="serial", serial_port="COM20")
# test_agent.hardware_init()

# test_agent.online_check()
# time.sleep(1)

# while True:
#     print(test_agent.get_all_position())
#     time.sleep(1)

# test_agent.move_all_init(1000, 50)
# time.sleep(1)

# test_agent.stand_up()
# time.sleep(1)

# while True:
#     test_agent.hw_go_forward()
