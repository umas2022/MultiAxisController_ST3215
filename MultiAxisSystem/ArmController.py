import sys

sys.path.append("..")


from MultiAxisSystem.MultiAxisController import MultiAxisController, MotorConfig


class ArmController(MultiAxisController):
    def __init__(self, serial_port):
        super().__init__(serial_port)
        self.motors_list = [
            MotorConfig(id=1, min=0, max=4096, init=2048, reverse=False),
            MotorConfig(id=2, min=0, max=4096, init=2048, reverse=False),
            MotorConfig(id=3, min=0, max=4096, init=2048, reverse=False),
            MotorConfig(id=4, min=0, max=4096, init=2048, reverse=False),
            MotorConfig(id=5, min=0, max=4096, init=2048, reverse=False),
            MotorConfig(id=6, min=0, max=4096, init=2048, reverse=False),
            MotorConfig(id=7, min=0, max=4096, init=2048, reverse=False),
        ]
        self.motors_num = len(self.motors_list)
