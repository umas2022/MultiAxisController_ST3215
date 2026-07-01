from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass
class MotorConfig:
    id: int
    min: int
    max: int
    init: int
    reverse: bool


class MultiAxisControllerInterface(ABC):
    @abstractmethod
    def hardware_init(self) -> bool:
        pass

    @abstractmethod
    def online_check(self) -> bool:
        pass

    @abstractmethod
    def move_to_absolute(self, motor: MotorConfig, position: int, speed: int, acc: int) -> bool:
        pass

    @abstractmethod
    def move_to_offset(self, motor: MotorConfig, offset: int, speed: int, acc: int) -> bool:
        pass

    @abstractmethod
    def move_excute(self) -> None:
        pass

    @abstractmethod
    def get_all_position(self) -> list:
        pass

    @abstractmethod
    def get_all_speed(self) -> list:
        pass

    @abstractmethod
    def get_all_load(self) -> list:
        pass

    @abstractmethod
    def get_all_temper(self) -> list:
        pass

    @abstractmethod
    def move_all_init(self, spd, acc) -> None:
        pass

    @abstractmethod
    def move_all_absolute(self, pos_list: list, spd_list: list, acc_list: list) -> None:
        pass

    @abstractmethod
    def move_all_offset(self, pos_list: list, spd_list: list, acc_list: list) -> None:
        pass

    def ping_motor(self, motor_id: int) -> bool:
        raise NotImplementedError("ping_motor is not implemented for this transport")

    def get_motor_position(self, motor_id: int):
        raise NotImplementedError("get_motor_position is not implemented for this transport")

    def set_motor_id(self, old_id: int, new_id: int) -> bool:
        raise NotImplementedError("set_motor_id is not implemented for this transport")

    def set_motor_zero(self, motor_id: int) -> bool:
        raise NotImplementedError("set_motor_zero is not implemented for this transport")

    def set_motor_torque(self, motor_id: int, enabled: bool) -> bool:
        raise NotImplementedError("set_motor_torque is not implemented for this transport")

    def set_wheel_mode(self, motor_id: int) -> bool:
        raise NotImplementedError("set_wheel_mode is not implemented for this transport")

    def set_motor_speed(self, motor_id: int, speed: int, acc: int = 0) -> bool:
        raise NotImplementedError("set_motor_speed is not implemented for this transport")
