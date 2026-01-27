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

        #   position:
        #     x: 0.16652227479993278
        #     y: 0.0007511309115031211
        #     z: 0.17405905738506888
        #   orientation:
        #     x: 0.0013651364485122808
        #     y: 0.7044913854184236
        #     z: 1.6116807232660296e-05
        #     w: 0.709711225791135

        # arm取牌 盒子顶部位置
        self.pickup_top_pos = [
            0.16652227479993278,
            0.0007511309115031211,
            0.17405905738506888,
            0.0013651364485122808,
            0.7044913854184236,
            0.000,
            0.709711225791135,
        ]

        #           position:
        #     x: 0.16624617701068495
        #     y: 0.0006438396845990931
        #     z: 0.20451618128131932
        #   orientation:
        #     x: 0.001587905318994773
        #     y: 0.7050192227845018
        #     z: -0.00021346829010272542
        #     w: 0.7091863848751779

        # arm取牌 盒子顶部 牌安全位置
        self.pickup_safe_pos = [
            0.16884652866283303,
            0.00035382640660968934,
            0.20451618128131932,
            0.002543809582243007,
            0.6964412346563269,
            0.0018569792845149434,
            0.7176069170033949,
        ]

        # arm 取牌和翻牌中间上位置
        self.mid_up_pos = [
            0.24095493768523785,
            0.000503924538811963,
            0.20451618128131932,
            0.0015087527248348644,
            0.705093516511921,
            -0.00014231802499460982,
            0.7091127106346695,
        ]

        # arm 翻牌上位置
        self.flip_up_pos = [
            0.3896457396332045,
            0.00021790326010464125,
            0.20451618128131932,
            0.0018070796363841345,
            0.7044133064943666,
            -0.00014243881199366246,
            0.7097877202432049,
        ]

        # arm 翻牌 放牌位置
        self.flip_drop_pos = [
            0.3896457396332045,
            0.00021790326010464125,
            0.144,
            0.0018070796363841345,
            0.7044133064943666,
            -0.00014243881199366246,
            0.7097877202432049,
        ]

        # ``        pose:
        # position:
        #     x: 0.3896457396332045
        #     y: 0.00021790326010464125
        #     z: 0.1252153364629985
        # orientation:
        #     x: 0.0018070796363841345
        #     y: 0.7044133064943666
        #     z: -0.00014243881199366246
        #     w: 0.7097877202432049``

        # arm 翻牌 吸牌位置
        self.flip_pickup_pos = [
            0.3896457396332045,
            0.00021790326010464125,
            0.1252153364629985,
            0.0018070796363841345,
            0.7044133064943666,
            -0.00014243881199366246,
            0.7097877202432049,
        ]

        # pose:
        # position:
        #     x: 0.24061702127093315
        #     y: 0.00045208216912801154
        #     z: 0.20507161061368168
        # orientation:
        #     x: 0.0014331587772337634
        #     y: 0.704942491563476
        #     z: -6.717545689516484e-05
        #     w: 0.7092630154827304

        # arm 存牌和翻牌中间上位置
        self.store_flip_pos = [
            0.24095493768523785,
            0.000503924538811963,
            0.20451618128131932,
            0.0015087527248348644,
            0.705093516511921,
            -0.00014231802499460982,
            0.7091127106346695,
        ]

        #         pose:
        # position:
        #     x: 0.24095493768523785
        #     y: 0.000503924538811963
        #     z: 0.17632378193536546
        # orientation:
        #     x: 0.0015087527248348644
        #     y: 0.705093516511921
        #     z: -0.00014231802499460982
        #     w: 0.7091127106346695

        # arm 存牌上位置
        self.store_up_pos = [
            0.24095493768523785,
            0.000503924538811963,
            0.17632378193536546,
            0.705093516511921,
            0.0015087527248348644,
            -0.00014231802499460982,
            0.7091127106346695,
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

    def move_to_predefined_position(
        self, position_var, position_name="predefined", tolerance=0.005, velocity=15.0
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
        self.set_min_velocity(velocity)

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
        self.get_logger().info(
            f"发布预定义姿态 {position_name}: "
            f"位置({x:.4f}, {y:.4f}, {z:.4f}), "
            f"方向({ox:.4f}, {oy:.4f}, {oz:.4f}, {ow:.4f})"
        )

        # 等待到达位置，允许1mm误差
        return self.wait_for_arrival(x, y, z, tolerance)

    def wait_for_arrival(
        self, target_x, target_y, target_z, tolerance=0.005, timeout=3
    ):
        """等待机械臂到达目标位置，增加超时控制"""
        self.get_logger().info(
            f"正在等待到达位置: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})"
        )

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
                    self.get_logger().info(f"已到达目标位置, 距离: {distance:.3f}")
                    return True
            else:
                # 记录我们没有收到位置数据
                self.get_logger().debug("等待接收末端执行器位置数据...")

            time.sleep(0.001)  # 减少等待间隔，从0.001改为0.005

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
        self, position_var, z_adjustment, tolerance=0.005, velocity=10.0
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
            position_var, "base_position", 0.01, velocity
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

        # 移动到翻牌位置
        # arm 翻牌上位置
        if not self.move_to_predefined_position(self.flip_up_pos, "flip_up_pos"):
            return False

        # arm 翻牌 放牌位置
        if not self.move_to_predefined_position(self.flip_drop_pos, "flip_drop_pos"):
            return False

        self.get_logger().info("关闭吸泵...")
        # 使用并行执行关闭吸泵
        pump_result = self.execute_parallel_tasks(
            (self.controller_app.pump_control, (1, False))
        )[0]

        # arm 翻牌上位置
        if not self.move_to_predefined_position(self.flip_up_pos, "flip_up_pos"):
            return False

        if not pump_result:
            self.get_logger().error("关闭吸泵失败")
            return False

        # 执行翻牌动作 - 使用并行执行
        self.get_logger().info("翻牌中...")
        slot_machine_result = self.execute_parallel_tasks(
            (self.controller_app.slot_machine_control, (True,))
        )[0]

        # arm 取牌和翻牌中间上位置
        if not self.move_to_predefined_position(self.mid_up_pos, "mid_up_pos"):
            return False

        if not self.take_picture():
            return False

        if not slot_machine_result:
            self.get_logger().error("翻牌失败")
            return False

        slot_machine_result = self.execute_parallel_tasks(
            (self.controller_app.slot_machine_control, (False,))
        )[0]

        if not slot_machine_result:
            self.get_logger().error("翻牌复位失败")
            return False

        # 启动吸泵 - 使用并行执行
        self.get_logger().info("启动吸泵...")
        pump_result = self.execute_parallel_tasks(
            (self.controller_app.pump_control, (1, True))
        )[0]

        if not self.take_picture():
            return False

        time.sleep(0.2)

        # arm 翻牌上位置
        if not self.move_to_predefined_position(self.flip_up_pos, "flip_up_pos"):
            return False

        # arm 翻牌吸牌位置
        if not self.adjust_end_effector_z_from_predefined(
            self.flip_pickup_pos, 0.05, tolerance=0.02
        ):
            self.get_logger().error("未能调整末端执行器Z轴位置")
            return False

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

            # 2. 开始取牌
            if not self.pickup_card():
                self.get_logger().error(f"第 {cycle + 1} 次循环取卡失败")
                break

            # 3. 移动到翻牌位置并翻牌
            if not self.flip_card():
                self.get_logger().error(f"第 {cycle + 1} 次循环翻牌失败")
                break

            # 4. 放置卡牌到存放位置
            if not self.drop_card():
                self.get_logger().error(f"第 {cycle + 1} 次循环放卡失败")
                break

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
            time.sleep(0.1)  # 减少等待间隔时间

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
