# pump_controller.py

import time
from modbus_client import ModbusClientWrapper


class PumpController:
    """
    泵与电磁阀控制器，封装 Modbus 控制逻辑。
    """

    def __init__(
        self, port="/dev/ttyACM0", baudrate=9600, parity="N", stopbits=1, bytesize=8
    ):
        self.client = ModbusClientWrapper(
            port=port,
            baudrate=baudrate,
            parity=parity,
            stopbits=stopbits,
            bytesize=bytesize,
        )

    def write_relay_io(self, relay_num: int, state: bool, relay_num_all=False):
        """控制单个或全部继电器，state：False-关闭，True-打开"""
        if relay_num_all:
            for i in range(16):
                self.client.add_read_write_coil_send(3, i, int(state))
            if self.client.get_recv_read_write_coil_data() != state:
                return False

        else:
            self.client.add_read_write_coil_send(3, relay_num, int(state))
            if self.client.get_recv_read_write_coil_data() != state:
                return False
        return True

    def pump_on(self):
        """打开泵，假设也接在 relay 0"""
        print("💦 泵已打开")
        return self.write_relay_io(15, True)

    def pump_off(self):
        """关闭泵"""
        print("💧 泵已关闭")
        return self.write_relay_io(15, False)

    def pump2_on(self):
        """打开泵，假设也接在 relay 0"""
        print("💦 泵2已打开")
        return self.write_relay_io(14, True)

    def pump2_off(self):
        """关闭泵"""
        print("💧 泵2已关闭")
        return self.write_relay_io(14, False)

    def pump_control(self, id, state: bool):
        """控制泵状态"""

        if id == 1:
            relay_num = 15
        elif id == 2:
            relay_num = 14
        return self.write_relay_io(relay_num, state)


if __name__ == "__main__":
    pump = PumpController()
    if not pump.pump_on():
        print("无法打开泵，请检查连接或配置。")
    time.sleep(3)
    if not pump.pump_off():
        print("无法关闭泵，请检查连接或配置。")
        exit(1)
