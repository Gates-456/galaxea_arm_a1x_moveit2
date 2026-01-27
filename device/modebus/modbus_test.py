# main.py

from modbus_client import ModbusClientWrapper
import time


def write_relay_io(modbus_client, relay_num, state, relay_num_all=False):
    """relay_num  1-16"""
    """state  0-关闭, 1-打开"""

    if relay_num_all:
        for i in range(0, 16):
            modbus_client.add_read_write_coil_send(3, i, state)
    else:
        modbus_client.add_read_write_coil_send(3, relay_num, state)


def pneumatic_solenoid_valve(modbus_client, state):
    """state  0-关闭, 1-打开"""
    write_relay_io(modbus_client, 0, state)
    print(f"气动电磁阀 {1} {'打开' if state else '关闭'}")


def on_pump(modbus_client):
    """打开泵"""
    write_relay_io(modbus_client, 0, True)
    print("泵已打开")


def off_pump(modbus_client):
    """关闭泵"""
    write_relay_io(modbus_client, 0, False)
    print("泵已关闭")


def hex_to_signed_decimal(decimal_value):

    # 判断是否为负数（16位有符号整数）
    if decimal_value[0] >= 32768:
        decimal_value[0] -= 65536  # 转换为负数

    return decimal_value


def main():
    modbus_client = ModbusClientWrapper(
        port="/dev/ttyLEFT", baudrate=9600, parity="N", stopbits=1, bytesize=8
    )

    modbus_client.start()

    try:
        # write_registers_state = modbus_client.add_write_registers_send(1, 1, 0)
        # if write_registers_state:
        #     print("请求添加成功")
        # else:
        #     print("请求添加失败")

        # success = modbus_client.add_read_input_registers_send(1, 0, 2)
        # if success:
        #     print("请求添加成功")
        # else:
        #     print("请求添加失败")

        # success = modbus_client.add_read_input_registers_send(2, 0, 2)
        # if success:
        #     print("请求添加成功")
        # else:
        #     print("请求添加失败")

        success_2 = modbus_client.add_read_input_registers_send(1, 1, 1)
        if success_2:
            print("请求添加成功 2")
        else:
            print("请求添加失败 2")

        time.sleep(1)

        received_data = modbus_client.get_received_data()
        print(f"接收到的数据tof: {received_data}")
        on_pump(modbus_client)
        time.sleep(0.2)
        off_pump(modbus_client)

        # success = modbus_client.add_read_holding_registers_send(1, 4, 1)
        # if success:
        #     print("请求添加成功")
        # else:
        #     print("请求添加失败")

        # time.sleep(1)

        # read_holding_data = modbus_client.get_read_holding_data()

        # a = hex_to_signed_decimal(read_holding_data)
        # print(f"接收到的数据kpa: {read_holding_data}  a: {a}")

        # if a[0] <= -5:
        #     print("压力过低")

    finally:
        modbus_client.stop()


if __name__ == "__main__":
    main()

    # modbus_client = ModbusClientWrapper(
    #     port="/dev/ttyACM0", baudrate=9600, parity="N", stopbits=1, bytesize=8
    # )
