# modbus_client.py

import threading
from pymodbus.client import ModbusSerialClient as ModbusClient
import time
import atexit

import logging

logger = logging.getLogger(__name__)


class ModbusClientWrapper:
    def __init__(self, port, baudrate, parity, stopbits, bytesize):
        # 初始化时不指定slave ID，因为ModbusSerialClient不支持该参数
        self.client = ModbusClient(
            port=port,
            baudrate=baudrate,
            parity=parity,
            stopbits=stopbits,
            bytesize=bytesize,
            timeout=1
        )

        self.stop_ = False

        self.running = True
        self.send_write_registers_data_ = []
        self.send_read_input_registers_data_ = []
        self.send_read_holding_registers_data_ = []
        self.send_read_write_coil_data_ = []

        self.recv_read_holding_registers_data_ = []
        self.recv_read_input_registers_data_ = []
        self.recv_read_write_coil_data_ = []

        self.read_thread = None
        self.write_thread = None

        # lock to protect serial client access and reconnect sequences
        self._modbus_lock = threading.Lock()

        # 在程序退出时自动释放资源，确保串口被关闭
        try:
            atexit.register(self.stop)
        except Exception:
            # 在极少数环境中注册可能失败，忽略以保证初始化不出错
            pass

    def _reconnect_client(self, attempts: int = 3, delay: float = 0.5) -> bool:
        """Try to reopen underlying serial client safely."""
        for i in range(attempts):
            try:
                logger.info(f"尝试重连串口 (尝试 {i+1}/{attempts})")
                try:
                    self.client.close()
                except Exception:
                    pass
                time.sleep(delay)
                
                # 重新连接后设置默认slave ID
                if self.client.connect():
                    logger.info("串口重连成功")
                    return True
            except Exception as e:
                logger.info(f"串口重连异常: {e}")
            time.sleep(delay * (i + 1))
        logger.info("串口重连多次失败")
        return False

    def start(self):
        # 尝试连接，失败则重试最多3次
        for attempt in range(3):
            connect_state = self.client.connect()
            if connect_state:
                break
            logger.info(f"无法连接到 Modbus 设备，重试 {attempt + 1}/3")
            self.client.close()
            time.sleep(0.5)
        else:
            logger.info("连续3次无法连接到 Modbus 设备，启动失败")
            return False

        self.read_thread = threading.Thread(target=self.read_modbus)
        self.write_thread = threading.Thread(target=self.write_registers_modebus)

        self.read_thread.daemon = True  # 设置为守护线程
        self.read_thread.start()
        self.write_thread.daemon = True  # 设置为守护线程
        self.write_thread.start()

    def stop(self):
        if not self.stop_:
            self.running = False

            if self.read_thread and self.read_thread.is_alive():
                self.read_thread.join()
            if self.write_thread and self.write_thread.is_alive():
                self.write_thread.join()
            self.client.close()
            self.stop_ = True

    def read_modbus(self):
        while self.running:
            self.read_input_registers_modbus()
            self.read_holding_registers_modbus()
            self.read_write_coil_modbus()

    def read_holding_registers_modbus(self):

        if self.send_read_holding_registers_data_:
            logger.info(
                f"当前待发送读取请求的数量: {len(self.send_read_holding_registers_data_)}"
            )
            start_register, address, count = self.send_read_holding_registers_data_.pop(
                0
            )
            logger.info(
                f"请求读取：地址={address}, 数量={count}, 从站={start_register}"
            )

            for attempt in range(3):

                try:
                    logger.info(f"尝试读取保持寄存器: {attempt }")
                    # 使用pymodbus的execute方法，直接传递unit参数
                    response = self.client.read_holding_registers(
                        address=address,
                        count=count,
                        slave=start_register
                    )

                except Exception as e:
                    logger.info(f"读取保持寄存器时发生错误: {e}")
                    logger.info("未收到响应，继续下一个请求")
                    break

                if not response:
                    logger.info(f"未收到响应，尝试 {attempt + 1}")

                if response.isError():
                    logger.info(f"读取保持寄存器错误: {response}, 尝试 {attempt + 1}")
                else:
                    self.recv_read_holding_registers_data_ = response.registers
                    logger.info(
                        f"保持寄存器数据: {self.recv_read_holding_registers_data_}"
                    )
                    break

                time.sleep(0.5)
            else:
                logger.info("未收到响应，继续下一个请求")

    def read_input_registers_modbus(self):
        if self.send_read_input_registers_data_:
            logger.info(
                f"当前待发送读取请求的数量: {len(self.send_read_input_registers_data_)}"
            )
            start_register, address, count = self.send_read_input_registers_data_.pop(0)
            logger.info(
                f"请求读取：地址={address}, 数量={count}, 从站={start_register}"
            )

            for attempt in range(3):
                try:
                    # 使用pymodbus的read_input_registers方法，通过slave参数指定从站
                    response = self.client.read_input_registers(
                        address=address,
                        count=count,
                        slave=start_register
                    )
                except Exception as e:
                    logger.info(f"读取输入寄存器时发生错误: {e}")
                    time.sleep(0.1)
                    continue

                if response.isError():
                    logger.info(f"读取输入寄存器错误: {response}, 尝试 {attempt + 1}")
                else:
                    self.recv_read_input_registers_data_ = response.registers
                    # logger.info(f"输入寄存器数据: {self.recv_read_input_registers_data_}")
                    if (
                        not self.recv_read_input_registers_data_
                        or len(self.recv_read_input_registers_data_) < 2
                    ):
                        logger.info(
                            "recv_read_input_registers_data_没有数据:",
                            self.recv_read_input_registers_data_,
                        )
                        continue
                    break

                time.sleep(0.1)
            else:
                logger.info("未收到响应，继续下一个请求")

    def read_write_coil_modbus(self):
        if self.send_read_write_coil_data_:
            start_register, address, value = self.send_read_write_coil_data_.pop(0)
            logger.info(
                f"请求读取：地址={address}, 数量={value}, 从站id={start_register}"
            )

            # 封装一次写入尝试，返回 True/False
            def try_write_once():
                try:
                    # 使用pymodbus的write_coil方法，通过slave参数指定从站
                    response = self.client.write_coil(
                        address=address,
                        value=value,
                        slave=start_register
                    )
                except PermissionError as pe:
                    logger.info(f"写入线圈时发生权限错误: {pe}")
                    return None, pe
                except Exception as e:
                    logger.info(f"写入线圈时发生异常: {e}")
                    return None, e

                if response is None:
                    return None, Exception("未收到响应")

                if hasattr(response, "isError") and response.isError():
                    return response, Exception(f"写入失败: {response}")

                # 成功
                status = True if getattr(response, "bits", [True])[0] else False
                logger.info(
                    f"成功写入线圈 {getattr(response, 'address', address)}: {status}"
                )
                self.recv_read_write_coil_data_ = status
                return response, None

            # 第1轮：尝试若干次
            for attempt in range(2):
                with self._modbus_lock:
                    resp, err = try_write_once()
                if err is None:
                    return
                # 权限错误时立即尝试重连并进入重连逻辑
                if isinstance(err, PermissionError):
                    logger.info("检测到 PermissionError，尝试重连串口...")
                    if self._reconnect_client():
                        break
                    else:
                        logger.info("重连失败，继续下一次尝试")
                else:
                    logger.info(f"写入尝试失败: {err}, 重试...")
                time.sleep(0.1)

            # 第二轮：尝试重连后再次写入
            logger.info("第一次重试失败，尝试重新打开串口并再次重试...")
            if not self._reconnect_client():
                logger.info("重连未成功，放弃本次写入请求")
                return

            for attempt in range(3):
                with self._modbus_lock:
                    resp, err = try_write_once()
                if err is None:
                    return
                logger.info(f"重连后写入尝试失败: {err}, 尝试 {attempt+1}/3")
                time.sleep(0.1)

            logger.info(
                "写入线圈连续重试失败，已重新打开串口仍无法写入，请检查硬件或连接。"
            )

    def write_registers_modebus(self):
        while self.running:
            if self.send_write_registers_data_:
                start_register, address, value = self.send_write_registers_data_.pop(0)
                
                # 使用正确的pymodbus API调用方式，传递unit参数指定从站
                for attempt in range(3):
                    write_response = self.client.write_registers(
                        address=address,
                        values=[value],
                        slave=start_register
                    )

                    if write_response.isError():
                        logger.info(f"写入错误: {write_response}, 尝试 {attempt + 1}")
                    else:
                        logger.info(
                            f"写入成功: {value} 到寄存器 {start_register}，地址 {address}"
                        )
                        break
                    time.sleep(0.1)
            time.sleep(0.001)

    def add_read_holding_registers_send(self, start_register, address, count):
        if (
            start_register,
            address,
            count,
        ) not in self.send_read_holding_registers_data_:
            self.send_read_holding_registers_data_.append(
                (start_register, address, count)
            )
            return True
        return False

    def add_read_input_registers_send(self, start_register, address, count):
        if (start_register, address, count) not in self.send_read_input_registers_data_:
            self.send_read_input_registers_data_.append(
                (start_register, address, count)
            )
            # logger.info(
            #     f"当前待发送读取请求的数量 111: {len(self.send_read_input_registers_data_)}"
            # )
            return True

        return False

    def add_read_write_coil_send(self, start_register, address, value):
        if (start_register, address, value) not in self.send_read_write_coil_data_:
            self.send_read_write_coil_data_.append((start_register, address, value))
            time.sleep(0.1)
            return True
        return False

    def add_write_registers_send(self, start_register, address, value):
        if (start_register, address, value) not in self.send_write_registers_data_:
            self.send_write_registers_data_.append((start_register, address, value))
            return True
        return False

    def get_received_data(self):
        return self.recv_read_input_registers_data_

    def get_read_holding_data(self):
        return self.recv_read_holding_registers_data_

    def get_recv_read_write_coil_data(self):
        return self.recv_read_write_coil_data_

    def hex_to_signed_decimal(self, hex_values):
        signed_decimals = []
        for hex_value in hex_values:
            # 将十六进制字符串转换为十进制整数
            decimal_value = int(hex_value, 16)

            # 判断是否为负数（16位有符号整数）
            if decimal_value >= 32768:
                decimal_value -= 65536  # 转换为负数

            signed_decimals.append(decimal_value)
        return signed_decimals

    def write_coil_sync(
        self, start_register, address, value, retries: int = 3, retry_delay: float = 0.1
    ):
        """Synchronous write_coil with lock, retries and optional reconnect.

        Returns: (success: bool, status: Optional[bool])
        - success: whether write succeeded
        - status: the coil status reported by slave (True/False) if available
        """
        # Ensure lock so no concurrent direct serial access
        for attempt in range(1, retries + 1):
            with self._modbus_lock:
                try:
                    # 使用pymodbus的write_coil方法，通过slave参数指定从站
                    response = self.client.write_coil(
                        address=address,
                        value=value,
                        slave=start_register
                    )
                except PermissionError as pe:
                    logger.info(f"同步写入线圈权限错误: {pe}")
                    # try reconnect once
                    if self._reconnect_client():
                        logger.info("同步写入重连成功，重试")
                        continue
                    return False, None
                except Exception as e:
                    logger.info(f"同步写入线圈异常: {e}")
                    time.sleep(retry_delay)
                    continue

                if response is None:
                    logger.info("同步写入未收到响应")
                    time.sleep(retry_delay)
                    continue

                if hasattr(response, "isError") and response.isError():
                    logger.info(f"同步写入返回错误: {response}")
                    time.sleep(retry_delay)
                    continue

                # 成功
                status = True if getattr(response, "bits", [True])[0] else False
                self.recv_read_write_coil_data_ = status
                return True, status

        return False, None

    def read_input_registers_sync(
        self,
        start_register,
        address,
        count,
        retries: int = 3,
        retry_delay: float = 0.05,
    ):
        """Synchronous read of input registers protected by lock.

        Returns: (success: bool, registers: Optional[List[int]])
        - success: whether read succeeded
        - registers: list of register values when success
        """
        for attempt in range(1, retries + 1):
            with self._modbus_lock:
                try:
                    # 使用pymodbus的read_input_registers方法，通过slave参数指定从站
                    response = self.client.read_input_registers(
                        address=address,
                        count=count,
                        slave=start_register
                    )
                except PermissionError as pe:
                    logger.info(f"同步读取输入寄存器权限错误: {pe}")
                    # 尝试重连一次
                    if self._reconnect_client():
                        logger.info("同步读重连成功，重试")
                        continue
                    return False, None
                except Exception as e:
                    logger.info(f"同步读取输入寄存器异常: {e}")
                    time.sleep(retry_delay)
                    continue

                if response is None:
                    logger.info("同步读取未收到响应")
                    time.sleep(retry_delay)
                    continue

                if hasattr(response, "isError") and response.isError():
                    logger.info(f"同步读取返回错误: {response}")
                    time.sleep(retry_delay)
                    continue

                # 成功
                registers = getattr(response, "registers", None)
                return True, registers

            # 若与锁外的重试间隔
            time.sleep(retry_delay)

        return False, None

    def read_holding_registers_sync(
        self,
        start_register,
        address,
        count,
        retries: int = 3,
        retry_delay: float = 0.05,
    ):
        """Synchronous read of holding registers protected by lock.

        Returns: (success: bool, registers: Optional[List[int]])
        - success: whether read succeeded
        - registers: list of register values when success
        """
        for attempt in range(1, retries + 1):
            with self._modbus_lock:
                # 临时修改slave ID
                old_slave = getattr(self.client, 'slave', 1)
                self.client.slave = start_register
                
                try:
                    # 使用pymodbus的read_holding_registers方法，通过slave参数指定从站
                    response = self.client.read_holding_registers(
                        address=address,
                        count=count,
                        slave=start_register
                    )
                except PermissionError as pe:
                    logger.info(f"同步读取保持寄存器权限错误: {pe}")
                    # 恢复slave ID
                    self.client.slave = old_slave
                    if self._reconnect_client():
                        logger.info("同步读重连成功，重试")
                        continue
                    return False, None
                except Exception as e:
                    logger.info(f"同步读取保持寄存器异常: {e}")
                    # 恢复slave ID
                    self.client.slave = old_slave
                    time.sleep(retry_delay)
                    continue

                # 恢复slave ID
                self.client.slave = old_slave

                if response is None:
                    logger.info("同步读取未收到响应")
                    time.sleep(retry_delay)
                    continue

                if hasattr(response, "isError") and response.isError():
                    logger.info(f"同步读取返回错误: {response}")
                    time.sleep(retry_delay)
                    continue

                registers = getattr(response, "registers", None)
                return True, registers

            time.sleep(retry_delay)

        return False, None