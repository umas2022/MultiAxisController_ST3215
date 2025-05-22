import sys

sys.path.append("..")

from MultiAxisSystem.MultiAxisController import MultiAxisController, MotorConfig


class DogController(MultiAxisController):
    def __init__(self, serial_port):
        super().__init__(serial_port)
        self.motors_list = [
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
        self.motors_num = len(self.motors_list)
