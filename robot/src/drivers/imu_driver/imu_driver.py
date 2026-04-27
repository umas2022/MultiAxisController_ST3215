# coding: UTF-8
import threading
import time
from abc import ABC, abstractmethod
from typing import Dict, Tuple

import serial

from src.drivers.imu_driver.lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from src.drivers.imu_driver.lib.device_model import DeviceModel
from src.drivers.imu_driver.lib.protocol_resolver.roles.wit_protocol_resolver import WitProtocolResolver

CMD_GET_IMU_ALL = 0x20


class IMUDriverInterface(ABC):
    @abstractmethod
    def hardware_init(self) -> bool:
        pass

    @abstractmethod
    def close(self) -> None:
        pass

    @abstractmethod
    def get_data(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
        pass

    @abstractmethod
    def wait_for_data(self, timeout_s: float = 2.0) -> bool:
        pass


class _IMUDataMixin:
    def __init__(self):
        self._latest_data: Dict[str, float] = {
            "accX": 0.0,
            "accY": 0.0,
            "accZ": 0.0,
            "gyroX": 0.0,
            "gyroY": 0.0,
            "gyroZ": 0.0,
            "angleX": 0.0,
            "angleY": 0.0,
            "angleZ": 0.0,
        }
        self._lock = threading.Lock()
        self._last_update_time = 0.0

    def get_data(self):
        with self._lock:
            acc = (self._latest_data["accX"], self._latest_data["accY"], self._latest_data["accZ"])
            gyro = (self._latest_data["gyroX"], self._latest_data["gyroY"], self._latest_data["gyroZ"])
            angle = (self._latest_data["angleX"], self._latest_data["angleY"], self._latest_data["angleZ"])
        return acc, gyro, angle

    def get_acc(self) -> Tuple[float, float, float]:
        return self.get_data()[0]

    def get_gyro(self) -> Tuple[float, float, float]:
        return self.get_data()[1]

    def get_angle(self) -> Tuple[float, float, float]:
        return self.get_data()[2]

    def wait_for_data(self, timeout_s: float = 2.0) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self._lock:
                if self._last_update_time > 0.0:
                    return True
            time.sleep(0.02)
        return False


class _BaseWitIMUDriver(_IMUDataMixin, IMUDriverInterface):
    def __init__(self, port: str, baud: int = 9600, dev_name: str = "HWT906P"):
        super().__init__()
        self.port = port
        self.baud = baud
        self.dev_name = dev_name

        JY901SDataProcessor.onVarChanged = []
        WitProtocolResolver.TempBytes = []
        WitProtocolResolver.TempFindValues = []

        self.device = DeviceModel(
            dev_name,
            WitProtocolResolver(),
            JY901SDataProcessor(),
            "51_0",
        )
        self.device.serialConfig.portName = port
        self.device.serialConfig.baud = baud
        self.device.dataProcessor.onVarChanged.append(self._on_update)

    def _on_update(self, imu_device: DeviceModel):
        with self._lock:
            for key in self._latest_data.keys():
                value = imu_device.getDeviceData(key)
                if value is not None:
                    self._latest_data[key] = value
            self._last_update_time = time.time()

    def hardware_init(self) -> bool:
        self.device.openDevice()
        return self.device.serialPort is not None and self.device.isOpen

    def start(self) -> bool:
        return self.hardware_init()

    def stop(self) -> None:
        self.close()

    def close(self) -> None:
        self.device.closeDevice()


class IMUDriverUSB(_BaseWitIMUDriver):
    """
    Direct IMU connection over a local USB serial port.
    """

    def __init__(self, port: str = "COM9", baud: int = 9600, dev_name: str = "HWT906P"):
        super().__init__(port=port, baud=baud, dev_name=dev_name)


class IMUDriverSerial(_IMUDataMixin, IMUDriverInterface):
    """
    IMU access through ESP32 Bluetooth/serial forwarding.
    """

    def __init__(self, port: str = "COM8", baud: int = 115200):
        super().__init__()
        self.port = port
        self.baud = baud
        self.port_handler = None

    def hardware_init(self) -> bool:
        try:
            self.port_handler = serial.Serial(self.port, baudrate=self.baud, timeout=0.2)
            return True
        except Exception as exc:
            print(f"Failed to open IMU serial bridge {self.port}: {exc}")
            self.port_handler = None
            return False

    def close(self) -> None:
        if self.port_handler is not None:
            try:
                if self.port_handler.is_open:
                    self.port_handler.close()
            except Exception:
                pass
        self.port_handler = None

    def _send_command(self, payload):
        if not self.port_handler or not self.port_handler.is_open:
            return
        checksum = sum(payload) & 0xFF
        frame = bytearray([0xAA] + payload + [checksum])
        self.port_handler.reset_input_buffer()
        self.port_handler.write(frame)

    def _read_response(self, expected_cmd: int, timeout_s: float = 0.5):
        if not self.port_handler or not self.port_handler.is_open:
            return None

        deadline = time.time() + timeout_s
        buffer = bytearray()

        while time.time() < deadline:
            waiting = self.port_handler.in_waiting
            if waiting:
                buffer.extend(self.port_handler.read(waiting))
                while len(buffer) >= 4:
                    if buffer[0] != 0xAA:
                        del buffer[0]
                        continue
                    payload_len = buffer[2]
                    frame_len = payload_len + 4
                    if len(buffer) < frame_len:
                        break
                    frame = bytes(buffer[:frame_len])
                    del buffer[:frame_len]
                    checksum = sum(frame[1:-1]) & 0xFF
                    if checksum != frame[-1]:
                        continue
                    if frame[1] == (expected_cmd | 0x80):
                        return frame
            time.sleep(0.01)

        return None

    @staticmethod
    def _to_int16(low: int, high: int) -> int:
        raw = (high << 8) | low
        if raw >= 0x8000:
            raw -= 0x10000
        return raw

    def _scale_and_store(self, raw_values):
        acc = [raw_values[i] / 32768.0 * 16.0 for i in range(3)]
        gyro = [raw_values[i + 3] / 32768.0 * 2000.0 for i in range(3)]
        angle = [raw_values[i + 6] / 32768.0 * 180.0 for i in range(3)]

        with self._lock:
            self._latest_data["accX"], self._latest_data["accY"], self._latest_data["accZ"] = acc
            self._latest_data["gyroX"], self._latest_data["gyroY"], self._latest_data["gyroZ"] = gyro
            self._latest_data["angleX"], self._latest_data["angleY"], self._latest_data["angleZ"] = angle
            self._last_update_time = time.time()

    def refresh(self, timeout_s: float = 0.5) -> bool:
        self._send_command([CMD_GET_IMU_ALL, 0x00])
        response = self._read_response(CMD_GET_IMU_ALL, timeout_s=timeout_s)
        if response is None or len(response) != 23:
            return False
        if response[3] != 0x01:
            return False

        raw_values = []
        for index in range(9):
            base = 4 + index * 2
            raw_values.append(self._to_int16(response[base], response[base + 1]))
        self._scale_and_store(raw_values)
        return True

    def wait_for_data(self, timeout_s: float = 2.0) -> bool:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.refresh(timeout_s=min(0.5, max(0.1, deadline - time.time()))):
                return True
        return False
