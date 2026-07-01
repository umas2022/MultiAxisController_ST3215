import numpy as np

from .MultiAxisController import MultiAxisControllerInterface, MotorConfig
from .STservo_sdk import COMM_SUCCESS, PortHandler, sts
from .STservo_sdk.sts import STS_ID, STS_TORQUE_ENABLE


class MultiAxisUSB(MultiAxisControllerInterface):
    """
    Multi-axis controller over direct USB servo bus.
    """

    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.baudrate = 1000000
        self.motors_list: list[MotorConfig] = []

        try:
            self.port_handler = PortHandler(self.serial_port)
        except Exception as exc:
            print(f"Failed to open the port: {exc}")
            raise

        self.packet_handler = sts(self.port_handler)

    def hardware_init(self) -> bool:
        if not self.port_handler.openPort():
            print("Failed to open the port")
            return False
        if not self.port_handler.setBaudRate(self.baudrate):
            print("Failed to change the baudrate")
            return False
        return True

    def ping_motor(self, motor_id: int) -> bool:
        _, sts_comm_result, sts_error = self.packet_handler.ping(motor_id)
        if sts_error != 0:
            print(self.packet_handler.getRxPacketError(sts_error))
        return sts_comm_result == COMM_SUCCESS

    def online_check(self) -> bool:
        offline_list = [motor.id for motor in self.motors_list if not self.ping_motor(motor.id)]
        if not offline_list:
            return True
        print(f"Offline motors: {offline_list}")
        return False

    def move_to_absolute(self, motor: MotorConfig, position: int, speed: int, acc: int) -> bool:
        range_check = True
        target_pos = int(np.clip(position, motor.min, motor.max))
        if target_pos != position:
            print(f"Target position {position} is out of range for motor {motor.id}.")
            range_check = False
        self.packet_handler.RegWritePosEx(motor.id, target_pos, speed, acc)
        return range_check

    def move_to_offset(self, motor: MotorConfig, offset: int, speed: int, acc: int) -> bool:
        if motor.reverse:
            offset = -offset
        return self.move_to_absolute(motor, motor.init + offset, speed, acc)

    def move_excute(self) -> None:
        self.packet_handler.RegAction()

    def move_all_init(self, spd: int, acc: int) -> None:
        for motor in self.motors_list:
            self.move_to_absolute(motor, motor.init, spd, acc)
        self.move_excute()

    def get_motor_position(self, motor_id: int):
        value, sts_comm_result, sts_error = self.packet_handler.ReadPos(motor_id)
        if sts_comm_result != COMM_SUCCESS:
            print(f"Motor {motor_id} is offline")
            return None
        if sts_error != 0:
            print(self.packet_handler.getRxPacketError(sts_error))
            return None
        return value

    def get_all_position(self) -> list:
        values = []
        for motor in self.motors_list:
            value = self.get_motor_position(motor.id)
            if value is None:
                return []
            values.append(value)
        return values

    def get_all_speed(self) -> list:
        values = []
        for motor in self.motors_list:
            value, sts_comm_result, sts_error = self.packet_handler.ReadSpeed(motor.id)
            if sts_comm_result != COMM_SUCCESS:
                print(f"Motor {motor.id} is offline")
                return []
            if sts_error != 0:
                print(self.packet_handler.getRxPacketError(sts_error))
                return []
            values.append(value)
        return values

    def get_all_load(self) -> list:
        values = []
        for motor in self.motors_list:
            value, sts_comm_result, sts_error = self.packet_handler.ReadLoad(motor.id)
            if sts_comm_result != COMM_SUCCESS:
                print(f"Motor {motor.id} is offline")
                return []
            if sts_error != 0:
                print(self.packet_handler.getRxPacketError(sts_error))
                return []
            values.append(value)
        return values

    def get_all_temper(self) -> list:
        values = []
        for motor in self.motors_list:
            value, sts_comm_result, sts_error = self.packet_handler.ReadTemprature(motor.id)
            if sts_comm_result != COMM_SUCCESS:
                print(f"Motor {motor.id} is offline")
                return []
            if sts_error != 0:
                print(self.packet_handler.getRxPacketError(sts_error))
                return []
            values.append(value)
        return values

    def get_all_position_load_temper(self) -> tuple:
        return self.get_all_position(), self.get_all_load(), self.get_all_temper()

    def set_motor_id(self, old_id: int, new_id: int) -> bool:
        sts_comm_result, sts_error = self.packet_handler.unLockEprom(old_id)
        if sts_comm_result != COMM_SUCCESS or sts_error != 0:
            return False
        sts_comm_result, sts_error = self.packet_handler.write1ByteTxRx(old_id, STS_ID, new_id)
        if sts_comm_result != COMM_SUCCESS or sts_error != 0:
            return False
        sts_comm_result, sts_error = self.packet_handler.LockEprom(new_id)
        if sts_comm_result != COMM_SUCCESS or sts_error != 0:
            return False
        return self.ping_motor(new_id)

    def set_motor_zero(self, motor_id: int) -> bool:
        sts_comm_result, sts_error = self.packet_handler.write1ByteTxRx(
            motor_id,
            STS_TORQUE_ENABLE,
            128,
        )
        return sts_comm_result == COMM_SUCCESS and sts_error == 0

    def set_motor_torque(self, motor_id: int, enabled: bool) -> bool:
        sts_comm_result, sts_error = self.packet_handler.write1ByteTxRx(
            motor_id,
            STS_TORQUE_ENABLE,
            1 if enabled else 0,
        )
        return sts_comm_result == COMM_SUCCESS and sts_error == 0

    def set_wheel_mode(self, motor_id: int) -> bool:
        sts_comm_result, sts_error = self.packet_handler.WheelMode(motor_id)
        return sts_comm_result == COMM_SUCCESS and sts_error == 0

    def set_motor_speed(self, motor_id: int, speed: int, acc: int = 0) -> bool:
        sts_comm_result, sts_error = self.packet_handler.WriteSpec(motor_id, speed, acc)
        return sts_comm_result == COMM_SUCCESS and sts_error == 0

    def move_all_absolute(self, pos_list: list, spd_list: list, acc_list: list) -> None:
        for index, motor in enumerate(self.motors_list):
            self.move_to_absolute(motor, pos_list[index], spd_list[index], acc_list[index])
        self.move_excute()

    def move_all_offset(self, pos_list: list, spd_list: list, acc_list: list) -> None:
        for index, motor in enumerate(self.motors_list):
            self.move_to_offset(motor, pos_list[index], spd_list[index], acc_list[index])
        self.move_excute()
