#!/usr/bin/env python3

"""
卡牌处理节点，实现取牌、放置、翻牌等功能
"""

import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Float64
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from math import sqrt
import sys
import os
from threading import Thread, Event
import concurrent.futures

# 添加路径以导入device包
sys.path.append("/home/robot/arm/a1x/a1x_ws/src/device/modebus")

# 尝试导入ControllerApp
try:
    from modbus_app import ControllerApp
except ImportError:
    # 如果上面的导入失败，尝试另一种方式
    try:
        from device.modebus.modbus_app import ControllerApp
    except ImportError:
        # 由于我们不使用模拟类，这里抛出错误
        raise ImportError("无法导入ControllerApp，请检查device/modebus模块是否正确安装")


class ArmCardDealerNode(Node):
    def __init__(self):
        super().__init__("arm_card_dealer_node")

        # 声明参数
        if not self.has_parameter("move_group_name"):
            self.declare_parameter("move_group_name", "a1x_group")
        if not self.has_parameter("end_effector_link"):
            self.declare_parameter("end_effector_link", "arm_link6")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        # 获取参数
        self.move_group_name = self.get_parameter("move_group_name").value
        self.end_effector_link = self.get_parameter("end_effector_link").value
        self.use_sim_time = self.get_parameter("use_sim_time").value

        # 初始化TF监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 创建末端执行器位置发布器
        self.target_pose_publisher = self.create_publisher(
            PoseStamped, "target_end_effector_pose", 10
        )

        # 订阅末端执行器当前位置
        self.current_pose_subscriber = self.create_subscription(
            PoseStamped, "end_effector_pose", self.pose_callback, 10
        )

        # 创建笛卡尔直线移动发布器
        self.cartesian_move_publisher = self.create_publisher(
            PoseStamped, "cartesian_linear_move", 10
        )

        # 创建Z轴直线移动发布器
        self.z_axis_move_publisher = self.create_publisher(
            PoseStamped, "z_axis_linear_move", 10
        )

        # 创建最小速度发布器
        self.min_velocity_publisher = self.create_publisher(
            Float64, "/min_velocity", 10
        )

        # 保存当前姿态
        self.current_pose = None
        self.last_pose_update_time = self.get_clock().now()

        # 初始化Modbus控制器
        self.controller_app = ControllerApp()

        # 预定义位置变量
        # arm 初始化 0的位置
        self.initial_zero_pos = [
            -0.0228,
            0.0000,
            0.2108,
            0.0000,
            0.0000,
            0.0000,
            1.0000,
        ]

        # arm 初始化 立起来的位置
        self.initial_upright_pos = [
            0.1568,
            -0.0000,
            0.3277,
            0.0000,
            0.6830,
            0.0000,
            0.7304,
        ]

        # arm取牌 盒子顶部位置
        self.pickup_top_pos = [
            0.16972437759547337,
            0.000545081363323365,
            0.17379604779380026,
            0.0027797264926409737,
            0.6968253596667089,
            0.0019444835942941453,
            0.717232814523666,
        ]

        # arm取牌 盒子顶部 牌安全位置
        self.pickup_safe_pos = [
            0.16884652866283303,
            0.00035382640660968934,
            0.1982,
            0.002543809582243007,
            0.6964412346563269,
            0.0018569792845149434,
            0.7176069170033949,
        ]

        # # arm 取牌和翻牌中间下位置
        # self.mid_down_pos = [0.1622, 0.0438, 0.1571, 0.0048, 0.7152, -0.0047, 0.6989]

        # arm 取牌和翻牌中间上位置
        self.mid_up_pos = [
            0.1628,
            0.0428,
            0.1982,
            0.0044,
            0.7147,
            -0.0040,
            0.6994,
        ]

        # arm 翻牌上位置
        self.flip_up_pos = [
            0.1644,
            0.1773,
            0.1982,
            0.0043,
            0.7149,
            -0.0040,
            0.6992,
        ]

        # arm 翻牌 放牌位置
        self.flip_drop_pos = [0.1633, 0.1770, 0.1318, 0.0052, 0.7147, -0.0040, 0.6994]

        # arm 翻牌 吸牌位置
        # self.flip_pickup_pos = [0.1630, 0.1770, 0.1193, 0.0050, 0.7145, -0.0039, 0.6996]
        self.flip_pickup_pos = [0.1630, 0.1770, 0.1253, 0.0050, 0.7145, -0.0039, 0.6996]

        # arm 存牌和翻牌中间上位置
        self.store_flip_pos = [
            0.23742337800445185,
            0.00022973308459775998,
            0.1982,
            0.002539835182532245,
            0.6967453095652594,
            0.001545902926008932,
            0.7177554579451046,
        ]

        # arm 存牌上位置
        self.store_up_pos = [
            0.23742337800445185,
            0.00022973308459775998,
            0.18276421217914102,
            0.002539835182532245,
            0.6967453095652594,
            0.001545902926008932,
            0.7177554579451046,
        ]

        self.get_logger().info(
            f"ArmCardDealerNode 初始化完成，组: {self.move_group_name}, "
            f"末端执行器: {self.end_effector_link}"
        )

    def execute_parallel_tasks(self, *tasks):
        """
        并行执行多个任务的通用函数

        Args:
            *tasks: 可调用对象的元组，格式为 (callable, args_tuple)

        Returns:
            list: 每个任务的返回结果
        """
        with concurrent.futures.ThreadPoolExecutor() as executor:
            # 提交所有任务到线程池
            futures = []
            for task_func, task_args in tasks:
                future = executor.submit(task_func, *task_args)
                futures.append(future)

            # 获取所有任务的结果
            results = []
            for future in concurrent.futures.as_completed(futures):
                results.append(future.result())

        return results

    def pose_callback(self, msg):
        """接收当前末端执行器位置的回调函数"""
        self.current_pose = msg.pose
        self.last_pose_update_time = self.get_clock().now()
        # 取消下面的调试日志，避免过多输出
        # self.get_logger().debug(f"接收到末端执行器位置: ({self.current_pose.position.x:.3f}, {self.current_pose.position.y:.3f}, {self.current_pose.position.z:.3f})")

    # 计算卡牌高度
    def calculate_robot_height(self, laser_measurement, deviation_mm=0):
        """
        根据激光测量值计算机器人高度

        参数:
            laser_measurement (float): 激光传感器测量值(mm)
            deviation_mm(float): 误差mm +向上 -向下

        返回:
            float: 机器人高度(m) 放到盒子顶部向下到卡牌位置的距离

        标定公式: H_robot = -1.04 * L_laser + 62
        其中:
            - 62mm: TOF传感器到盒子顶部的距离
            - 65mm: 机械臂末端吸泵长度
            - 最终结果转换为米
        """
        # 参数验证
        if laser_measurement is None:
            self.get_logger().error("激光测量值不能为None")
            raise ValueError("激光测量值不能为None")

        # 标定参数
        CALIBRATION_SLOPE = -1.04
        TOF_TO_BOX_DISTANCE = 62  # mm
        # END_EFFECTOR_LENGTH = 65  # mm

        # 计算并返回结果
        robot_height_mm = (
            CALIBRATION_SLOPE * laser_measurement
            + TOF_TO_BOX_DISTANCE
            # + END_EFFECTOR_LENGTH
        ) + deviation_mm
        return robot_height_mm / 1000.0  # 转换为米

    def set_min_velocity(self, velocity):
        """设置机械臂最小速度"""

        msg = Float64()
        msg.data = velocity
        self.min_velocity_publisher.publish(msg)
        self.get_logger().info(f"发布最小速度: {velocity}")
        return True

    def move_to_predefined_position(
        self, position_var, position_name="predefined", tolerance=0.03, velocity=52.0
    ):
        """
        移动到预定义位置

        Args:
            position_var: 预定义位置变量，包含[x, y, z, ox, oy, oz, ow]的列表
            position_name: 位置名称，用于日志记录
        """
        if len(position_var) != 7:
            self.get_logger().error(
                f"无效的位置变量 {position_name}: 期望7个值 [x,y,z,ox,oy,oz,ow]"
            )
            return False

        # 设置机械臂最小速度
        # self.set_min_velocity(velocity)

        x, y, z, ox, oy, oz, ow = position_var

        # 创建带有姿态的目标位置消息
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.x = ox
        pose_msg.pose.orientation.y = oy
        pose_msg.pose.orientation.z = oz
        pose_msg.pose.orientation.w = ow

        self.target_pose_publisher.publish(pose_msg)
        # self.get_logger().info(
        #     f"发布预定义姿态 {position_name}: "
        #     f"位置({x:.4f}, {y:.4f}, {z:.4f}), "
        #     f"方向({ox:.4f}, {oy:.4f}, {oz:.4f}, {ow:.4f})"
        # )

        # 等待到达位置，允许1mm误差
        return self.wait_for_arrival(x, y, z, tolerance)

    def wait_for_arrival(
        self, target_x, target_y, target_z, tolerance=0.005, timeout=3
    ):
        """等待机械臂到达目标位置，增加超时控制"""
        # self.get_logger().info(
        #     f"正在等待到达位置: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})"
        # )

        start_time = time.time()

        while time.time() - start_time < timeout:
            if self.current_pose is not None:
                # 检查位置数据是否过时（超过1秒）
                current_time = self.get_clock().now()
                time_diff = (
                    current_time - self.last_pose_update_time
                ).nanoseconds / 1e9
                if time_diff > 1.0:  # 如果数据超过1秒，视为过时
                    self.get_logger().warn(
                        f"位置数据已过时 {time_diff:.2f} 秒，可能move_to_pose节点未发布更新"
                    )
                    time.sleep(0.001)  # 等待一点时间再检查
                    continue  # 继续等待新的位置数据

                distance = sqrt(
                    (self.current_pose.position.x - target_x) ** 2
                    + (self.current_pose.position.y - target_y) ** 2
                    + (self.current_pose.position.z - target_z) ** 2
                )

                if distance <= tolerance:
                    # self.get_logger().info(f"已到达目标位置, 距离: {distance:.3f}")
                    return True
            else:
                # 记录我们没有收到位置数据
                self.get_logger().debug("等待接收末端执行器位置数据...")

            time.sleep(0.0001)  # 减少等待间隔，从0.001改为0.005

        # 如果等待超时，仍然返回True以继续执行，而不是中断流程
        if self.current_pose is not None:
            current_pos_str = f"({self.current_pose.position.x:.3f}, {self.current_pose.position.y:.3f}, {self.current_pose.position.z:.3f})"
            time_diff_str = f", 数据时差: {time_diff:.2f}s"
            self.get_logger().warn(
                f"等待到达 ({target_x:.3f}, {target_y:.3f}, {target_z:.3f}) 超时，当前位置: {current_pos_str}{time_diff_str} 误差: {distance}, 允许误差：{tolerance}，为避免长时间等待将继续下一步"
            )
        else:
            self.get_logger().warn(f"位置数据不可用，继续下一步")

        return True  # 即使超时也继续执行，避免长时间等待

    def adjust_end_effector_z_from_predefined(
        self,
        position_var,
        z_adjustment,
        tolerance_1=0.01,
        tolerance=0.005,
        velocity=52.0,
    ):
        """
        从预定义位置开始，对末端执行器Z轴进行微调

        Args:
            position_var: 预定义位置变量，包含[x, y, z, ox, oy, oz, ow]的列表
            z_adjustment: Z轴调整量（米），正值为上升，负值为下降
            step_size: 每步移动距离（米），默认0.005m (5mm)
        """
        if len(position_var) != 7:
            self.get_logger().error("无效的位置变量: 期望7个值 [x,y,z,ox,oy,oz,ow]")
            return False

        # 先移动到预定义的基础位置
        success = self.move_to_predefined_position(
            position_var, "base_position", tolerance_1, velocity
        )
        if not success:
            self.get_logger().error("移动到预定义位置失败")
            return False

        # 从预定义位置提取z值
        x, y, z, ox, oy, oz, ow = position_var

        # 计算最终Z轴位置
        target_z = z + z_adjustment

        # 检查Z轴是否低于安全距离
        min_safe_z = 0.06  # 最小安全距离0.06米
        if target_z < min_safe_z:
            self.get_logger().warn(
                f"目标Z轴位置 {target_z:.3f}m 低于安全距离 {min_safe_z}m，调整为安全距离"
            )
            target_z = min_safe_z

        # 使用笛卡尔直线移动发布器，发送绝对位置
        cartesian_move_msg = PoseStamped()
        cartesian_move_msg.header = Header()
        cartesian_move_msg.header.stamp = self.get_clock().now().to_msg()
        cartesian_move_msg.header.frame_id = "base_link"
        # 设置绝对位置坐标
        cartesian_move_msg.pose.position.x = x
        cartesian_move_msg.pose.position.y = y
        cartesian_move_msg.pose.position.z = target_z
        # 保持原始姿态不变
        cartesian_move_msg.pose.orientation.x = ox
        cartesian_move_msg.pose.orientation.y = oy
        cartesian_move_msg.pose.orientation.z = oz
        cartesian_move_msg.pose.orientation.w = ow

        self.cartesian_move_publisher.publish(cartesian_move_msg)
        self.get_logger().info(
            f"发布笛卡尔直线移动命令: 绝对位置Z轴调整至 {target_z:.3f}m"
        )

        # 等待移动完成，检查是否到达目标位置
        return self.wait_for_arrival(
            x, y, target_z, tolerance, timeout=2
        )  # Z轴调整时间更短

    def send_initial_position(self):
        """发送初始化位置"""
        self.get_logger().info("移动到初始位置...")
        if self.move_to_predefined_position(
            self.initial_upright_pos, "initial_upright_pos"
        ):
            return True
        else:
            return False

    def pickup_card(self):
        """执行取卡动作"""
        self.get_logger().info("开始取卡过程...")

        # 在移动到安全位置的过程中，并行检查是否有卡牌
        tof_data = self.execute_parallel_tasks((self.controller_app.get_tof_data, ()))[
            0
        ]

        # 启动吸泵 - 使用并行执行
        self.get_logger().info("启动吸泵...")
        pump_result = self.execute_parallel_tasks(
            (self.controller_app.pump_control, (1, True))
        )[0]

        # 先移动到安全位置
        if not self.move_to_predefined_position(
            self.pickup_safe_pos, "pickup_safe_pos"
        ):
            return False

        if tof_data is not None:
            self.get_logger().info(f"TOF距离: {tof_data} mm")

            # 如果tof_data大于125mm或小于60mm，则认为无牌
            if tof_data > 125 or tof_data < 60:
                self.get_logger().info(f"TOF距离:{tof_data},超出范围，无牌")
                return False
            # 没有牌了，退出循环
            else:
                self.get_logger().info(f"TOF距离:{tof_data},在范围内，有牌")

                # 计算机器人高度
                pos_z = self.calculate_robot_height(tof_data)
                self.get_logger().info(f"计算得到的机械臂高度: {pos_z:.3f} m")
        else:
            self.get_logger().error("无法获取TOF距离数据")
            return False

        card_surface_height = pos_z

        if not pump_result:
            self.get_logger().error("启动吸泵失败")
            return False

        # arm取牌 盒子顶部位置
        if not self.adjust_end_effector_z_from_predefined(
            self.pickup_top_pos,
            card_surface_height,
        ):
            self.get_logger().error("未能调整末端执行器Z轴位置")
            return False

        # arm取牌 盒子顶部 牌安全位置
        if not self.move_to_predefined_position(
            self.pickup_safe_pos, "pickup_safe_pos"
        ):
            return False

        return True

    # 拍照
    def take_picture(self):
        # 拍照（这里只是模拟，实际需要集成摄像头）
        self.get_logger().info("翻牌后拍照...")
        time.sleep(0.1)  # 减少拍照等待时间，从0.3s到0.1s
        return True

    def flip_card(self):
        """执行翻牌动作"""
        self.get_logger().info("移动到翻牌位置...")

        # 移动到翻牌位置 - 计时开始
        flip_up_start_time = time.time()
        # arm 翻牌上位置
        if not self.move_to_predefined_position(self.flip_up_pos, "flip_up_pos"):
            self.get_logger().error("移动到翻牌上位置失败")
            return False
        flip_up_duration = time.time() - flip_up_start_time
        self.get_logger().info(
            f"✓ 移动到翻牌上位置完成，耗时: {flip_up_duration:.2f} 秒"
        )

        # 移动到放牌位置 - 计时
        flip_drop_start_time = time.time()
        # arm 翻牌 放牌位置
        if not self.move_to_predefined_position(self.flip_drop_pos, "flip_drop_pos"):
            self.get_logger().error("移动到翻牌放牌位置失败")
            return False
        flip_drop_duration = time.time() - flip_drop_start_time
        self.get_logger().info(
            f"✓ 移动到翻牌放牌位置完成，耗时: {flip_drop_duration:.2f} 秒"
        )

        # 关闭吸泵 - 计时
        pump_off_start_time = time.time()
        self.get_logger().info("关闭吸泵...")
        # 使用并行执行关闭吸泵
        pump_result = self.execute_parallel_tasks(
            (self.controller_app.pump_control, (1, False))
        )[0]
        pump_off_duration = time.time() - pump_off_start_time
        self.get_logger().info(f"✓ 关闭吸泵完成，耗时: {pump_off_duration:.2f} 秒")

        # 回到翻牌上位置 - 计时
        flip_up_return_start_time = time.time()
        # arm 翻牌上位置
        if not self.move_to_predefined_position(self.flip_up_pos, "flip_up_pos"):
            self.get_logger().error("回到翻牌上位置失败")
            return False
        flip_up_return_duration = time.time() - flip_up_return_start_time
        self.get_logger().info(
            f"✓ 回到翻牌上位置完成，耗时: {flip_up_return_duration:.2f} 秒"
        )

        if not pump_result:
            self.get_logger().error("关闭吸泵失败")
            return False

        # 执行翻牌动作 - 计时
        slot_machine_start_time = time.time()
        self.get_logger().info("翻牌中...")
        slot_machine_result = self.execute_parallel_tasks(
            (self.controller_app.slot_machine_control, (True,))
        )[0]
        slot_machine_duration = time.time() - slot_machine_start_time
        self.get_logger().info(f"✓ 翻牌动作完成，耗时: {slot_machine_duration:.2f} 秒")

        # 移动到中间位置 - 计时
        mid_up_start_time = time.time()
        # arm 取牌和翻牌中间上位置
        if not self.move_to_predefined_position(self.mid_up_pos, "mid_up_pos"):
            self.get_logger().error("移动到中间上位置失败")
            return False
        mid_up_duration = time.time() - mid_up_start_time
        self.get_logger().info(
            f"✓ 移动到中间上位置完成，耗时: {mid_up_duration:.2f} 秒"
        )

        # 拍照 - 计时
        picture1_start_time = time.time()
        if not self.take_picture():
            self.get_logger().error("第一次拍照失败")
            return False
        picture1_duration = time.time() - picture1_start_time
        self.get_logger().info(f"✓ 第一次拍照完成，耗时: {picture1_duration:.2f} 秒")

        if not slot_machine_result:
            self.get_logger().error("翻牌失败")
            return False

        # 翻牌复位 - 计时
        slot_reset_start_time = time.time()
        slot_machine_result = self.execute_parallel_tasks(
            (self.controller_app.slot_machine_control, (False,))
        )[0]
        slot_reset_duration = time.time() - slot_reset_start_time
        self.get_logger().info(f"✓ 翻牌复位完成，耗时: {slot_reset_duration:.2f} 秒")

        if not slot_machine_result:
            self.get_logger().error("翻牌复位失败")
            return False

        # 启动吸泵 - 计时
        pump_on_start_time = time.time()
        # 启动吸泵 - 使用并行执行
        self.get_logger().info("启动吸泵...")
        pump_result = self.execute_parallel_tasks(
            (self.controller_app.pump_control, (1, True))
        )[0]
        pump_on_duration = time.time() - pump_on_start_time
        self.get_logger().info(f"✓ 启动吸泵完成，耗时: {pump_on_duration:.2f} 秒")

        # 第二次拍照 - 计时
        picture2_start_time = time.time()
        if not self.take_picture():
            self.get_logger().error("第二次拍照失败")
            return False
        picture2_duration = time.time() - picture2_start_time
        self.get_logger().info(f"✓ 第二次拍照完成，耗时: {picture2_duration:.2f} 秒")

        # 延时 - 计时
        delay_start_time = time.time()
        time.sleep(0.1)
        delay_duration = time.time() - delay_start_time
        self.get_logger().info(f"✓ 延时完成，耗时: {delay_duration:.2f} 秒")

        # 再次移动到翻牌上位置 - 计时
        flip_up_second_start_time = time.time()
        # arm 翻牌上位置
        if not self.move_to_predefined_position(self.flip_up_pos, "flip_up_pos"):
            self.get_logger().error("再次移动到翻牌上位置失败")
            return False
        flip_up_second_duration = time.time() - flip_up_second_start_time
        self.get_logger().info(
            f"✓ 再次移动到翻牌上位置完成，耗时: {flip_up_second_duration:.2f} 秒"
        )

        # 移动到翻牌吸牌位置 - 计时
        flip_pickup_start_time = time.time()
        # arm 翻牌吸牌位置
        if not self.adjust_end_effector_z_from_predefined(
            self.flip_pickup_pos, 0.05, tolerance_1=0.02, tolerance=0.015
        ):
            self.get_logger().error("未能调整末端执行器Z轴位置")
            return False
        flip_pickup_duration = time.time() - flip_pickup_start_time
        self.get_logger().info(
            f"✓ 调整末端执行器Z轴位置完成，耗时: {flip_pickup_duration:.2f} 秒"
        )

        if not pump_result:
            self.get_logger().error("启动吸泵失败")
            return False

        self.get_logger().info("翻牌成功完成")
        return True

    def drop_card(self):
        """放置卡牌到存放位置"""
        self.get_logger().info("移动到放置位置...")

        # arm 存牌和翻牌中间上位置
        if not self.move_to_predefined_position(self.store_flip_pos, "store_flip_pos"):
            return False

        # arm 存牌上位置
        if not self.move_to_predefined_position(self.store_up_pos, "store_up_pos"):
            return False

        # 关闭吸泵释放卡牌 - 使用并行执行
        self.get_logger().info("释放卡牌...")
        pump_result = self.execute_parallel_tasks(
            (self.controller_app.pump_control, (1, False))
        )[0]

        if not pump_result:
            self.get_logger().error("关闭吸泵失败")
            return False

        self.get_logger().info("卡牌放置成功")
        return True

    def get_current_pose(self):
        """获取当前末端执行器的姿态"""
        return self.current_pose

    def run_card_handling_process(self):

        # 记录开始时间
        start_time = time.time()
        self.get_logger().info("开始卡牌处理流程，记录开始时间")

        # 循环执行10次或直到没有卡牌
        for cycle in range(1):
            self.get_logger().info(f"开始第 {cycle + 1}/10 次循环")

            # 开始取牌
            pickup_start_time = time.time()
            self.get_logger().info("开始取牌步骤")
            if not self.pickup_card():
                self.get_logger().error(f"第 {cycle + 1} 次循环取卡失败")
                break
            else:
                pickup_duration = time.time() - pickup_start_time
                self.get_logger().info(
                    f"✓ 取牌步骤完成，耗时: {pickup_duration:.2f} 秒"
                )

            # 移动到翻牌位置并翻牌
            flip_start_time = time.time()
            self.get_logger().info("开始翻牌步骤")
            if not self.flip_card():
                self.get_logger().error(f"第 {cycle + 1} 次循环翻牌失败")
                break
            else:
                flip_duration = time.time() - flip_start_time
                self.get_logger().info(f"✓ 翻牌步骤完成，耗时: {flip_duration:.2f} 秒")

            # 放置卡牌到存放位置
            drop_start_time = time.time()
            self.get_logger().info("开始放牌步骤")
            if not self.drop_card():
                self.get_logger().error(f"第 {cycle + 1} 次循环放卡失败")
            else:
                drop_duration = time.time() - drop_start_time
                self.get_logger().info(f"✓ 放牌步骤完成，耗时: {drop_duration:.2f} 秒")

            self.get_logger().info(f"完成第 {cycle + 1} 次循环")

        # 记录结束时间并计算总耗时
        end_time = time.time()
        total_duration = end_time - start_time
        self.get_logger().info(f"卡牌处理流程完成，总耗时: {total_duration:.2f} 秒")

        # 回到初始位置
        # if not self.send_initial_position():
        #     self.get_logger().error(f"未发送初始位置")
        #     return False
        self.get_logger().info("卡牌处理流程完成")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ArmCardDealerNode()
        node.get_logger().info("ArmCardDealerNode 已启动.")

        # 使用线程运行ROS消息处理
        spin_thread = Thread(target=lambda n=node: rclpy.spin(n), args=(node,))
        spin_thread.daemon = True
        spin_thread.start()

        # 等待接收位置数据
        timeout = time.time() + 60 * 2  # 2分钟超时
        while node.current_pose is None and time.time() < timeout:
            node.get_logger().info("等待接收末端执行器位置数据...")
            time.sleep(0.01)  # 减少等待间隔时间

        if node.current_pose is None:
            node.get_logger().error(
                "未能接收末端执行器位置数据，可能move_to_pose节点未运行"
            )
            return

        # 运行卡牌处理流程
        node.run_card_handling_process()

        node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
