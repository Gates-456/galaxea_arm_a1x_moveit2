import time
import logging

# 修复导入方式 - 由于是在当前目录下运行，使用相对导入
from modbus_client import ModbusClientWrapper

logger = logging.getLogger(__name__)


class ControllerApp:
    """
    封装 Modbus app 控制逻辑。
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
        # 启动客户端
        self.client.start()

    def write_relay_io(self, relay_num: int, state: bool, relay_num_all=False):
        """控制单个或全部继电器，state：False-关闭，True-打开

        Returns:
            bool: 操作是否成功
        """
        if relay_num_all:
            success_count = 0
            for i in range(16):
                # 修复API调用，添加正确的start_register参数
                success, _ = self.client.write_coil_sync(3, i, int(state))
                if success:
                    success_count += 1

            # 检查是否所有操作都成功
            result = success_count == 16
            logger.info(f"批量设置继电器状态: {state}, 成功 {success_count}/16")
            return result
        else:
            # 修复API调用，添加正确的start_register参数
            success, _ = self.client.write_coil_sync(3, relay_num, int(state))
            logger.info(
                f"设置继电器 {relay_num} 状态: {state}, 结果: {'成功' if success else '失败'}"
            )
            return success

    def pump_on(self):
        """打开泵，接在 relay 15"""
        logger.info("打开泵")
        success = self.write_relay_io(15, True)
        if success:
            print("💦 泵已打开")
        else:
            print("❌ 泵打开失败")
        return success

    def pump_off(self):
        """关闭泵"""
        logger.info("关闭泵")
        success = self.write_relay_io(15, False)
        if success:
            print("💧 泵已关闭")
        else:
            print("❌ 泵关闭失败")
        return success

    def pump2_on(self):
        """打开泵2，接在 relay 14"""
        logger.info("打开泵2")
        success = self.write_relay_io(14, True)
        if success:
            print("💦 泵2已打开")
        else:
            print("❌ 泵2打开失败")
        return success

    def pump2_off(self):
        """关闭泵2"""
        logger.info("关闭泵2")
        success = self.write_relay_io(14, False)
        if success:
            print("💧 泵2已关闭")
        else:
            print("❌ 泵2关闭失败")
        return success

    def pump_control(self, id, state: bool):
        """控制泵状态

        Args:
            id (int): 泵ID (1 或 2)
            state (bool): 状态 (True-打开, False-关闭)

        Returns:
            bool: 操作是否成功
        """
        if id == 1:
            relay_num = 15
        elif id == 2:
            relay_num = 14
        else:
            logger.error(f"不支持的泵ID: {id}")
            return False

        return self.write_relay_io(relay_num, state)

    def slot_machine_control(self, state: bool):
        """控制翻牌机状态

        Args:
            slot_id (int): 翻牌机ID (0-15)
            state (bool): 状态 (True-打开, False-关闭)

        Returns:
            bool: 操作是否成功
        """
        slot_id = 0  # 假设翻牌机接在 relay 0
        if 0 <= slot_id <= 15:
            return self.write_relay_io(slot_id, state)
        else:
            logger.error(f"不支持的翻牌机ID: {slot_id}")
            return False

    def get_tof_data(self):
        """获取TOF传感器数据（使用同步 Modbus 读取以提高可靠性）

        Returns:
            float: TOF距离(mm)，如果获取失败返回None
        """
        MAX_RETRIES = 3
        for attempt in range(1, MAX_RETRIES + 1):
            try:
                # 使用同步读取，避免异步队列/响应不匹配
                # 修复API调用，添加正确的start_register参数
                ok, registers = self.client.read_input_registers_sync(
                    2, 0, 2, retries=3
                )
                logger.debug(
                    f"TOF 同步读取尝试 {attempt}/{MAX_RETRIES}, ok={ok}, regs={registers}"
                )

                if not ok or not registers:
                    logger.warning(f"TOF 同步读取无效 (尝试 {attempt}/{MAX_RETRIES})")
                    time.sleep(0.05)
                    continue

                if len(registers) < 2:
                    logger.warning(f"TOF 返回寄存器长度不足: {registers}")
                    time.sleep(0.05)
                    continue

                tof_mm = registers[1] / 100.0

                if tof_mm == 0:
                    logger.info(f"TOF没有检测到物体 (尝试 {attempt}/{MAX_RETRIES})")
                    time.sleep(0.05)
                    continue

                logger.debug(f"TOF读取成功: {tof_mm:.2f} mm")
                return tof_mm

            except Exception as e:
                logger.error(f"TOF数据获取异常 (尝试 {attempt}/{MAX_RETRIES}): {e}")
                time.sleep(0.05)

        logger.error("TOF数据获取最终失败")
        return None


if __name__ == "__main__":
    # 配置基本的日志格式
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )

    app = ControllerApp()
    tof_data = app.get_tof_data()
    if tof_data is not None:
        print("TOF距离:", tof_data)
    else:
        print("无法获取TOF距离数据")

    app.slot_machine_control(True)
    time.sleep(0.2)
    app.slot_machine_control(False)

    app.pump_control(1, True)
    app.pump_control(1, False)
