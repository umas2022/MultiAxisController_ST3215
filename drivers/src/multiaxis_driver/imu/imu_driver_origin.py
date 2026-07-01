# coding: UTF-8
from .imu_driver import IMUDriverUSB


class IMUDriver(IMUDriverUSB):
    """
    Backward-compatible IMU driver alias.
    """

    def __init__(self, port: str = "COM9", baud: int = 9600, dev_name: str = "HWT906P"):
        super().__init__(port=port, baud=baud, dev_name=dev_name)
