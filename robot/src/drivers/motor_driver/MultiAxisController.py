import sys
from abc import ABC, abstractmethod
from dataclasses import dataclass



from src.drivers.motor_driver.STservo_sdk import *


@dataclass
class MotorConfig:
    id: int
    min: int
    max: int
    init: int
    reverse: bool


class MultiAxisControllerInterface(ABC):
    """
    多轴控制器接口
    """

    @abstractmethod
    def hardware_init(self) -> bool:
        """
        硬件初始化
        """
        pass

    @abstractmethod
    def online_check(self) -> bool:
        """
        检查所有电机在线
        """
        pass

    @abstractmethod
    def move_to_absolute(self, motor: MotorConfig, position: int, speed: int, acc: int) -> bool:
        """
        Move to absolute position
        """
        pass

    @abstractmethod
    def move_to_offset(self, motor: MotorConfig, offset: int, speed: int, acc: int) -> bool:
        """
        Move to offset position
        """
        pass

    @abstractmethod
    def move_excute(self) -> None:
        """
        Execute the move command
        """
        pass

    @abstractmethod
    def get_all_position(self) -> list:
        """
        获取所有电机位置
        """
        pass

    @abstractmethod
    def get_all_load(self) -> list:
        """
        获取所有电机负载
        """
        pass

    @abstractmethod
    def get_all_temper(self) -> list:
        """
        获取所有电机温度
        """
        pass

    @abstractmethod
    def move_all_init(self, spd, acc) -> None:
        """
        驱动所有电机到初始位置（中点）
        """
        pass

    @abstractmethod
    def move_all_absolute(self, pos_list: list, spd_list: list, acc_list: list) -> None:
        """
        驱动所有电机到给定位置（输入绝对位置）
        """
        pass

    @abstractmethod
    def move_all_offset(self, pos_list: list, spd_list: list, acc_list: list) -> None:
        """
        驱动所有电机到给定位置（输入相对于中点的偏移量）
        """
        pass
