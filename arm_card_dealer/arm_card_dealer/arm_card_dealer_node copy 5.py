#!/usr/bin/env python3

"""
卡牌处理节点优化版 - 提升响应速度
"""

import rclpy
from rclpy.node import Node
import time
import threading
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header, Float64, Bool
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from math import sqrt, fabs
import sys
import os
from threading import Thread, Event

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

        # 创建Z轴直线移动发布器
        self.z_axis_move_publisher = self.create_publisher(
            PoseStamped, "z_axis_linear_move", 10
        )

        # 创建最小速度发布器
        self.min_velocity_publisher = self.create_publisher(
            Float64, "/min_velocity", 10
        )

        # 创建运动状态订阅器
        self.motion_status_subscriber = self.create_subscription(
            Bool, "/motion_complete", self.motion_status_callback, 10
        )

        # 保存当前姿态
        self.current_pose = None
        self.last_pose_update_time = self.get_clock().now()
        self.motion_complete = False
        self.motion_lock = threading.Lock()

        # 初始化Modbus控制器
        self.controller_app = ControllerApp()

        # 预定义位置变量（优化版，减少中间点）
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

        # arm取牌 盒子顶部 牌安全位置（直接从此位置下降取牌）
        self.pickup_safe_pos = [
            0.1620,
            -0.0012,
            0.1706,
            0.0042,
            0.7151,
            -0.0042,
            0.6990,
        ]

        # 取牌后直接移动到中间点（简化路径）
        self.mid_pos = [0.1625, 0.0433, 0.1721, 0.0045, 0.7149, -0.0043, 0.6992]

        # 翻牌上位置
        self.flip_up_pos = [0.1644, 0.1773, 0.1871, 0.0043, 0.7149, -0.0040, 0.6992]

        # 翻牌 放牌位置
        self.flip_drop_pos = [0.1633, 0.1770, 0.1318, 0.0052, 0.7147, -0.0040, 0.6994]

        # 存牌上位置
        self.store_up_pos = [0.2293, -0.0001, 0.1456, 0.0043, 0.7150, -0.0041, 0.6991]

        # 快速移动速度（提高速度参数）
        self.fast_velocity = 10.0  # 提高速度
        self.slow_velocity = 1.0  # 精确定位时使用

        self.get_logger().info(f"ArmCardDealerNode 初始化完成（优化版）")

    def pose_callback(self, msg):
        """接收当前末端执行器位置的回调函数"""
        self.current_pose = msg.pose
        self.last_pose_update_time = self.get_clock().now()

    def motion_status_callback(self, msg):
        """运动完成状态回调"""
        with self.motion_lock:
            self.motion_complete = msg.data
            if self.motion_complete:
                self.get_logger().debug("检测到运动完成")

    # 计算卡牌高度
    def calculate_robot_height(self, laser_measurement, deviation_mm=0):
        """
        根据激光测量值计算机器人高度
        """
        if laser_measurement is None:
            self.get_logger().error("激光测量值不能为None")
            raise ValueError("激光测量值不能为None")

        # 标定参数
        CALIBRATION_SLOPE = -1.04
        TOF_TO_BOX_DISTANCE = 62  # mm

        robot_height_mm = (
            CALIBRATION_SLOPE * laser_measurement + TOF_TO_BOX_DISTANCE
        ) + deviation_mm
        return robot_height_mm / 1000.0  # 转换为米

    def set_min_velocity(self, velocity):
        """设置机械臂最小速度"""
        msg = Float64()
        msg.data = velocity
        self.min_velocity_publisher.publish(msg)
        self.get_logger().debug(f"设置速度: {velocity}")

    def move_to_position(
        self,
        position,
        position_name="",
        tolerance=0.01,
        velocity=None,
        check_arrival=True,
    ):
        """
        移动到指定位置（优化版）

        Args:
            position: 位置列表 [x, y, z, ox, oy, oz, ow]
            position_name: 位置名称
            tolerance: 位置容差，默认1cm（非关键位置可增大）
            velocity: 移动速度，None时使用fast_velocity
            check_arrival: 是否检查到达目标位置
        """
        if len(position) != 7:
            self.get_logger().error(f"无效的位置: 期望7个值")
            return False

        # 设置速度
        if velocity is None:
            velocity = self.fast_velocity
        self.set_min_velocity(velocity)

        x, y, z, ox, oy, oz, ow = position

        # 创建目标位置消息
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

        # 重置运动完成标志
        with self.motion_lock:
            self.motion_complete = False

        self.target_pose_publisher.publish(pose_msg)
        self.get_logger().info(f"发布位置: {position_name}, {position}")

        if check_arrival:
            # 使用较短超时和较大容差
            return self.wait_for_arrival_optimized(x, y, z, tolerance, timeout=10.0)
        else:
            # 不等待到达，立即返回（用于路径点）
            time.sleep(0.1)  # 短暂等待确保消息发送
            return True

    def wait_for_arrival_optimized(
        self, target_x, target_y, target_z, tolerance=0.01, timeout=10.0
    ):
        """优化版等待到达目标位置"""
        self.get_logger().debug(
            f"等待到达: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})"
        )

        start_time = time.time()
        last_log_time = start_time
        last_distance = float("inf")

        # 初始等待，让机械臂开始移动
        time.sleep(0.05)

        while time.time() - start_time < timeout:
            # 检查运动完成标志
            with self.motion_lock:
                if self.motion_complete:
                    self.get_logger().debug("运动完成标志触发")
                    # 最终位置验证
                    if self.current_pose is not None:
                        distance = sqrt(
                            (self.current_pose.position.x - target_x) ** 2
                            + (self.current_pose.position.y - target_y) ** 2
                            + (self.current_pose.position.z - target_z) ** 2
                        )
                        if distance <= tolerance:
                            return True

            # 使用当前位置检查
            if self.current_pose is not None:
                distance = sqrt(
                    (self.current_pose.position.x - target_x) ** 2
                    + (self.current_pose.position.y - target_y) ** 2
                    + (self.current_pose.position.z - target_z) ** 2
                )

                # 动态日志：当距离显著变化时记录
                if time.time() - last_log_time > 0.5:  # 每0.5秒记录一次
                    if abs(distance - last_distance) > 0.005:  # 距离变化大于5mm才记录
                        self.get_logger().debug(
                            f"当前距离: {distance:.4f}, 目标: {tolerance}"
                        )
                        last_log_time = time.time()
                        last_distance = distance

                # 到达检查
                if distance <= tolerance:
                    self.get_logger().debug(f"已到达, 距离: {distance:.4f}")
                    return True

                # 检查是否在靠近（防止卡住）
                if distance < 0.02:  # 距离很近但可能未达到容差
                    if fabs(self.current_pose.position.z - target_z) < tolerance:
                        # Z轴已到达，XY稍有偏差也可以接受
                        return True

            time.sleep(0.01)  # 较短等待

        # 超时处理
        if self.current_pose is not None:
            distance = sqrt(
                (self.current_pose.position.x - target_x) ** 2
                + (self.current_pose.position.y - target_y) ** 2
                + (self.current_pose.position.z - target_z) ** 2
            )
            self.get_logger().warn(
                f"等待超时, 当前位置: ({self.current_pose.position.x:.3f}, "
                f"{self.current_pose.position.y:.3f}, {self.current_pose.position.z:.3f}), "
                f"距离: {distance:.4f}, "
                f"目标: {tolerance}"
            )
        return False

    def move_z_relative(self, z_adjustment, tolerance=0.005, velocity=0.2):
        """Z轴相对移动"""
        # 设置速度
        self.set_min_velocity(velocity)

        # 使用Z轴直线移动
        z_move_msg = PoseStamped()
        z_move_msg.header = Header()
        z_move_msg.header.stamp = self.get_clock().now().to_msg()
        z_move_msg.header.frame_id = "base_link"
        z_move_msg.pose.position.z = z_adjustment

        # 重置运动完成标志
        with self.motion_lock:
            self.motion_complete = False

        self.z_axis_move_publisher.publish(z_move_msg)
        self.get_logger().debug(f"Z轴移动: {z_adjustment:.3f}m")

        # 等待移动完成
        if self.current_pose is not None:
            target_z = self.current_pose.position.z + z_adjustment
            return self.wait_for_arrival_optimized(
                self.current_pose.position.x,
                self.current_pose.position.y,
                target_z,
                tolerance,
                timeout=5.0,
            )
        return True

    def send_initial_position(self):
        """发送初始化位置"""
        self.get_logger().info("移动到初始位置...")
        return self.move_to_position(
            self.initial_upright_pos,
            "initial_upright_pos",
            tolerance=0.015,  # 较大容差
            velocity=self.fast_velocity,
        )

    def pickup_card(self, card_surface_height):
        """执行取卡动作（优化版）"""
        self.get_logger().info("开始取卡...")

        # 1. 移动到安全位置（快速）
        if not self.move_to_position(
            self.pickup_safe_pos,
            "pickup_safe_pos",
            tolerance=0.015,
            velocity=self.fast_velocity,
        ):
            return False

        # 2. 启动吸泵（与下降并行）
        self.get_logger().info("启动吸泵...")
        pump_thread = threading.Thread(
            target=self.controller_app.pump_control, args=(1, True)
        )
        pump_thread.start()

        # 3. 下降到取牌位置（较慢，精确）
        # 计算目标Z位置
        target_z = self.pickup_safe_pos[2] - card_surface_height
        min_safe_z = 0.06
        if target_z < min_safe_z:
            self.get_logger().warn(f"调整Z位置到安全高度: {min_safe_z}")
            target_z = min_safe_z

        # 直接移动到目标位置（避免多次调整）
        pickup_pos = self.pickup_safe_pos.copy()
        pickup_pos[2] = target_z

        if not self.move_to_position(
            pickup_pos,
            "pickup_pos",
            tolerance=0.005,  # 较小容差，精确
            velocity=self.slow_velocity,
        ):
            return False

        # 等待吸泵完全启动
        pump_thread.join(timeout=1.0)
        time.sleep(0.1)  # 短暂确保吸住

        # 4. 上升到安全位置
        if not self.move_to_position(
            self.pickup_safe_pos,
            "pickup_safe_pos_up",
            tolerance=0.01,
            velocity=self.slow_velocity,
        ):
            return False

        # 5. 移动到中间位置
        return self.move_to_position(
            self.mid_pos,
            "mid_pos",
            tolerance=0.02,  # 较大容差，路径点
            velocity=self.fast_velocity,
            check_arrival=False,  # 不严格检查到达
        )

    def flip_card(self):
        """执行翻牌动作（优化版）"""
        self.get_logger().info("翻牌中...")

        # 1. 移动到翻牌上方（快速）
        if not self.move_to_position(
            self.flip_up_pos, "flip_up_pos", tolerance=0.02, velocity=self.fast_velocity
        ):
            return False

        # 2. 下降到放牌位置
        if not self.move_to_position(
            self.flip_drop_pos,
            "flip_drop_pos",
            tolerance=0.005,  # 精确
            velocity=self.slow_velocity,
        ):
            return False

        # 3. 关闭吸泵放牌
        self.get_logger().info("放牌...")
        if not self.controller_app.pump_control(1, False):
            self.get_logger().error("关闭吸泵失败")

        time.sleep(0.1)  # 确保卡牌释放

        # 4. 回到翻牌上方
        if not self.move_to_position(
            self.flip_up_pos,
            "flip_up_pos_return",
            tolerance=0.01,
            velocity=self.fast_velocity,
        ):
            return False

        # 5. 执行翻牌机构动作
        self.get_logger().info("执行翻牌机构...")
        self.controller_app.slot_machine_control(True)
        time.sleep(0.1)
        self.controller_app.slot_machine_control(False)

        # 6. 拍照（模拟）
        self.get_logger().info("拍照...")
        time.sleep(0.2)

        # 7. 启动吸泵准备吸牌
        self.get_logger().info("启动吸泵...")
        pump_thread = threading.Thread(
            target=self.controller_app.pump_control, args=(1, True)
        )
        pump_thread.start()

        # 8. 下降到吸牌位置（精确）
        pickup_z = self.flip_drop_pos[2] + 0.01  # 稍微高于放牌位置
        pickup_pos = self.flip_drop_pos.copy()
        pickup_pos[2] = pickup_z

        if not self.move_to_position(
            pickup_pos,
            "flip_pickup",
            tolerance=0.003,
            velocity=self.slow_velocity,  # 高精度  # 很慢
        ):
            return False

        # 等待吸泵
        pump_thread.join(timeout=0.5)
        time.sleep(0.2)  # 确保吸住

        # 9. 回到翻牌上方
        return self.move_to_position(
            self.flip_up_pos,
            "flip_up_pos_final",
            tolerance=0.01,
            velocity=self.slow_velocity,
        )

    def drop_card(self):
        """放置卡牌到存放位置（优化版）"""
        self.get_logger().info("放置卡牌...")

        # 1. 直接移动到存放位置（直线路径）
        # 计算中间路径点，使运动更平滑
        current_pos = [
            self.flip_up_pos[0],
            self.flip_up_pos[1],
            self.flip_up_pos[2],
            self.flip_up_pos[3],
            self.flip_up_pos[4],
            self.flip_up_pos[5],
            self.flip_up_pos[6],
        ]

        # 逐步移动到存放位置（分两段：先水平，再垂直）
        # 第一段：保持高度，移动到X位置
        mid_pos = current_pos.copy()
        mid_pos[0] = (current_pos[0] + self.store_up_pos[0]) / 2
        mid_pos[1] = (current_pos[1] + self.store_up_pos[1]) / 2

        if not self.move_to_position(
            mid_pos,
            "store_mid",
            tolerance=0.03,  # 大容差
            velocity=self.fast_velocity,
            check_arrival=False,
        ):
            return False

        time.sleep(0.05)  # 短暂间隔

        # 第二段：移动到最终位置
        if not self.move_to_position(
            self.store_up_pos,
            "store_up_pos",
            tolerance=0.01,
            velocity=self.slow_velocity,
        ):
            return False

        # 2. 关闭吸泵放牌
        self.get_logger().info("释放卡牌...")
        if not self.controller_app.pump_control(1, False):
            self.get_logger().warn("关闭吸泵失败，继续执行")

        time.sleep(0.1)  # 确保卡牌释放

        return True

    def run_card_handling_process(self):
        """运行完整的卡牌处理流程（优化版）"""
        self.get_logger().info("开始卡牌处理流程...")

        # 发送初始化位置
        if not self.send_initial_position():
            self.get_logger().error("初始化失败")
            return False

        # 循环执行
        max_cycles = 10
        for cycle in range(max_cycles):
            self.get_logger().info(f"=== 第 {cycle + 1}/{max_cycles} 次循环 ===")

            # 1. 检查是否有卡牌
            try:
                tof_data = self.controller_app.get_tof_data()
                if tof_data is None:
                    self.get_logger().error("TOF传感器无数据")
                    continue

                self.get_logger().info(f"TOF距离: {tof_data} mm")

                # 检查是否有牌
                if tof_data > 125 or tof_data < 60:
                    self.get_logger().info(f"无卡牌 (TOF: {tof_data}mm)")
                    break

            except Exception as e:
                self.get_logger().error(f"TOF读取失败: {e}")
                time.sleep(1)
                continue

            # 2. 计算取牌高度
            try:
                pos_z = self.calculate_robot_height(
                    tof_data, deviation_mm=-1
                )  # 轻微向下偏移确保吸住
                self.get_logger().info(f"取牌高度: {pos_z:.3f}m")
            except Exception as e:
                self.get_logger().error(f"高度计算失败: {e}")
                continue

            # 3. 取牌
            start_time = time.time()
            if not self.pickup_card(pos_z):
                self.get_logger().error("取牌失败")
                continue
            self.get_logger().info(f"取牌完成，耗时: {time.time()-start_time:.2f}s")

            # 4. 翻牌
            start_time = time.time()
            if not self.flip_card():
                self.get_logger().error("翻牌失败")
                continue
            self.get_logger().info(f"翻牌完成，耗时: {time.time()-start_time:.2f}s")

            # 5. 放置
            start_time = time.time()
            if not self.drop_card():
                self.get_logger().error("放置失败")
                continue
            self.get_logger().info(f"放置完成，耗时: {time.time()-start_time:.2f}s")

            # 6. 短暂停顿，准备下一次
            if cycle < max_cycles - 1:  # 不是最后一次循环
                self.get_logger().info("准备下一次取牌...")
                # 快速回到中间位置
                if not self.move_to_position(
                    self.mid_pos,
                    "prep_next",
                    tolerance=0.03,
                    velocity=self.fast_velocity,
                    check_arrival=False,
                ):
                    self.get_logger().warn("准备位置移动失败")

        # 回到初始位置
        self.get_logger().info("流程完成，回到初始位置...")
        # self.send_initial_position()

        self.get_logger().info("卡牌处理流程全部完成")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ArmCardDealerNode()
        node.get_logger().info("ArmCardDealerNode 已启动.")

        # 使用线程运行ROS消息处理
        spin_thread = Thread(target=lambda n=node: rclpy.spin(n), args=(node,))
        spin_thread.daemon = True
        spin_thread.start()

        # 等待接收位置数据（较短超时）
        timeout = time.time() + 30  # 30秒超时
        while node.current_pose is None and time.time() < timeout:
            node.get_logger().info("等待位置数据...")
            time.sleep(0.5)

        if node.current_pose is None:
            node.get_logger().error("未收到位置数据")
            return

        # 运行卡牌处理流程
        start_time = time.time()
        node.run_card_handling_process()
        total_time = time.time() - start_time
        node.get_logger().info(f"总执行时间: {total_time:.2f}秒")

        node.destroy_node()
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    except Exception as e:
        node.get_logger().error(f"程序异常: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
