#!/usr/bin/env python3

"""
卡牌处理节点，实现取牌、放置、翻牌等功能
优化版 - 重新设计并行策略
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
from queue import Queue
import traceback

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
            PoseStamped, "target_end_effector_pose", 100
        )

        # 订阅末端执行器当前位置
        self.current_pose_subscriber = self.create_subscription(
            PoseStamped, "end_effector_pose", self.pose_callback, 10
        )

        # 创建笛卡尔直线移动发布器
        self.cartesian_move_publisher = self.create_publisher(
            PoseStamped, "cartesian_linear_move", 100
        )

        # 创建Z轴直线移动发布器
        self.z_axis_move_publisher = self.create_publisher(
            PoseStamped, "z_axis_linear_move", 100
        )

        # 创建最小速度发布器
        self.min_velocity_publisher = self.create_publisher(
            Float64, "/min_velocity", 10
        )

        # 保存当前姿态
        self.current_pose = None
        self.last_pose_update_time = self.get_clock().now()
        self.pose_ready_event = Event()  # 用于同步

        # 注意：这里不能使用executor作为变量名，因为会与Node的executor属性冲突
        self.thread_pool = concurrent.futures.ThreadPoolExecutor(max_workers=4)

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

        # 性能统计
        self.stats = {
            "total_cycles": 0,
            "total_time": 0,
            "pickup_time": 0,
            "flip_time": 0,
            "drop_time": 0,
            "parallel_ops": 0,
            "parallel_saved_time": 0,
        }

        self.get_logger().info(
            f"ArmCardDealerNode 初始化完成，组: {self.move_group_name}, "
            f"末端执行器: {self.end_effector_link}"
        )

    def execute_truly_parallel(self, tasks_with_names):
        """
        真正的并行执行 - 只用于可以独立并行执行的任务

        Args:
            tasks_with_names: [(任务名, 任务函数, 参数元组), ...]

        Returns:
            dict: {任务名: 结果}
        """
        start_time = time.time()
        results = {}
        futures = {}

        # 提交所有任务
        for task_name, task_func, task_args in tasks_with_names:
            future = self.thread_pool.submit(task_func, *task_args)
            futures[task_name] = future
            self.get_logger().debug(f"已提交并行任务: {task_name}")

        # 收集所有结果
        for task_name, future in futures.items():
            try:
                results[task_name] = future.result(timeout=2.0)  # 每个任务最长等待2秒
                self.get_logger().debug(f"任务 {task_name} 完成")
            except concurrent.futures.TimeoutError:
                self.get_logger().warn(f"任务 {task_name} 超时")
                results[task_name] = None
            except Exception as e:
                self.get_logger().error(f"任务 {task_name} 出错: {str(e)}")
                results[task_name] = None

        duration = time.time() - start_time
        self.stats["parallel_ops"] += 1
        self.stats["parallel_saved_time"] += duration  # 实际节省的时间

        return results

    def pose_callback(self, msg):
        """接收当前末端执行器位置的回调函数"""
        self.current_pose = msg.pose
        self.last_pose_update_time = self.get_clock().now()
        self.pose_ready_event.set()

    def calculate_robot_height(self, laser_measurement, deviation_mm=0):
        """
        根据激光测量值计算机器人高度
        """
        if laser_measurement is None:
            self.get_logger().error("激光测量值不能为None")
            raise ValueError("激光测量值不能为None")

        CALIBRATION_SLOPE = -1.04
        TOF_TO_BOX_DISTANCE = 62  # mm

        robot_height_mm = (
            CALIBRATION_SLOPE * laser_measurement + TOF_TO_BOX_DISTANCE
        ) + deviation_mm
        return robot_height_mm / 1000.0

    def set_min_velocity(self, velocity):
        """设置机械臂最小速度"""
        msg = Float64()
        msg.data = velocity
        self.min_velocity_publisher.publish(msg)
        self.get_logger().debug(f"发布最小速度: {velocity}")
        return True

    def move_to_predefined_position(
        self, position_var, position_name="predefined", tolerance=0.045, velocity=52.0
    ):
        """移动到预定义位置"""
        if len(position_var) != 7:
            self.get_logger().error(
                f"无效的位置变量 {position_name}: 期望7个值 [x,y,z,ox,oy,oz,ow]"
            )
            return False

        x, y, z, ox, oy, oz, ow = position_var

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

        return self.wait_for_arrival(x, y, z, tolerance)

    def wait_for_arrival(
        self, target_x, target_y, target_z, tolerance=0.005, timeout=10
    ):
        """等待机械臂到达目标位置"""
        start_time = time.time()

        while time.time() - start_time < timeout:
            if self.current_pose is not None:
                current_time = self.get_clock().now()
                time_diff = (
                    current_time - self.last_pose_update_time
                ).nanoseconds / 1e9
                if time_diff > 1.0:
                    time.sleep(0.001)
                    continue

                distance = sqrt(
                    (self.current_pose.position.x - target_x) ** 2
                    + (self.current_pose.position.y - target_y) ** 2
                    + (self.current_pose.position.z - target_z) ** 2
                )

                if distance <= tolerance:
                    return True
            time.sleep(0.001)

        # 超时处理
        if self.current_pose is not None:
            time_diff = (
                self.get_clock().now() - self.last_pose_update_time
            ).nanoseconds / 1e9
            distance = sqrt(
                (self.current_pose.position.x - target_x) ** 2
                + (self.current_pose.position.y - target_y) ** 2
                + (self.current_pose.position.z - target_z) ** 2
            )
            self.get_logger().warn(
                f"等待到达超时，误差: {distance:.3f}，允许误差：{tolerance}"
            )
        return True

    def adjust_end_effector_z_from_predefined(
        self,
        position_var,
        z_adjustment,
        tolerance_1=0.01,
        tolerance=0.005,
        velocity=52.0,
    ):
        """从预定义位置开始，对末端执行器Z轴进行微调"""
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

        x, y, z, ox, oy, oz, ow = position_var
        target_z = z + z_adjustment

        min_safe_z = 0.06
        if target_z < min_safe_z:
            self.get_logger().warn(
                f"目标Z轴位置 {target_z:.3f}m 低于安全距离 {min_safe_z}m，调整为安全距离"
            )
            target_z = min_safe_z

        cartesian_move_msg = PoseStamped()
        cartesian_move_msg.header = Header()
        cartesian_move_msg.header.stamp = self.get_clock().now().to_msg()
        cartesian_move_msg.header.frame_id = "base_link"
        cartesian_move_msg.pose.position.x = x
        cartesian_move_msg.pose.position.y = y
        cartesian_move_msg.pose.position.z = target_z
        cartesian_move_msg.pose.orientation.x = ox
        cartesian_move_msg.pose.orientation.y = oy
        cartesian_move_msg.pose.orientation.z = oz
        cartesian_move_msg.pose.orientation.w = ow

        self.cartesian_move_publisher.publish(cartesian_move_msg)
        self.get_logger().info(
            f"发布笛卡尔直线移动命令: 绝对位置Z轴调整至 {target_z:.3f}m"
        )

        return self.wait_for_arrival(x, y, target_z, tolerance, timeout=2)

    def send_initial_position(self):
        """发送初始化位置"""
        self.get_logger().info("移动到初始位置...")
        return self.move_to_predefined_position(
            self.initial_upright_pos, "initial_upright_pos"
        )

    def pickup_card(self):
        """执行取卡动作 - 优化并行策略"""
        start_time = time.time()
        self.get_logger().info("开始取卡过程...")

        try:
            # 策略1: 在移动过程中并行执行设备操作
            # 先启动吸泵和获取TOF数据，同时开始移动
            move_future = self.thread_pool.submit(
                self.move_to_predefined_position,
                self.pickup_safe_pos,
                "pickup_safe_pos",
                tolerance=0.068,
            )

            # 并行执行设备操作
            device_tasks = [
                ("get_tof", self.controller_app.get_tof_data, ()),
                ("start_pump", self.controller_app.pump_control, (1, True)),
            ]
            device_results = self.execute_truly_parallel(device_tasks)

            # 等待移动完成
            move_result = move_future.result(timeout=5.0)
            if not move_result:
                self.get_logger().error("移动到安全位置失败")
                return False

            tof_data = device_results.get("get_tof")
            pump_result = device_results.get("start_pump")

            if not pump_result:
                self.get_logger().error("启动吸泵失败")
                return False

            if tof_data is not None:
                self.get_logger().info(f"TOF距离: {tof_data} mm")

                if tof_data > 125 or tof_data < 60:
                    self.get_logger().info(f"TOF距离:{tof_data},超出范围，无牌")
                    return False
                else:
                    self.get_logger().info(f"TOF距离:{tof_data},在范围内，有牌")
                    pos_z = self.calculate_robot_height(tof_data)
                    self.get_logger().info(f"计算得到的机械臂高度: {pos_z:.3f} m")

                    # 调整到取牌位置
                    if not self.adjust_end_effector_z_from_predefined(
                        self.pickup_top_pos, pos_z
                    ):
                        self.get_logger().error("未能调整末端执行器Z轴位置")
                        return False

                    # 返回安全位置
                    if not self.move_to_predefined_position(
                        self.pickup_safe_pos, "pickup_safe_pos", tolerance=0.048
                    ):
                        return False
            else:
                self.get_logger().error("无法获取TOF距离数据")
                return False

            duration = time.time() - start_time
            self.stats["pickup_time"] += duration
            self.get_logger().info(f"✓ 取卡过程完成，耗时: {duration:.2f} 秒")
            return True

        except Exception as e:
            self.get_logger().error(
                f"取卡过程发生异常: {str(e)}\n{traceback.format_exc()}"
            )
            return False

    def take_picture(self):
        """拍照（模拟）"""
        self.get_logger().debug("拍照...")
        time.sleep(0.1)  # 进一步减少拍照时间
        return True

    def flip_card(self):
        """执行翻牌动作 - 优化并行策略"""
        start_time = time.time()
        self.get_logger().info("开始翻牌动作...")

        try:
            # 1. 移动到翻牌上位置
            if not self.move_to_predefined_position(
                self.flip_up_pos, "flip_up_pos", tolerance=0.06
            ):
                return False

            # 2. 移动到放牌位置
            if not self.move_to_predefined_position(
                self.flip_drop_pos, "flip_drop_pos"
            ):
                return False

            # 3. 关闭吸泵（立即执行，不等待）
            pump_off_future = self.thread_pool.submit(
                self.controller_app.pump_control, 1, False
            )

            # 4. 立即开始移动回翻牌上位置（与关闭吸泵并行）
            if not self.move_to_predefined_position(self.flip_up_pos, "flip_up_pos"):
                return False

            # 等待吸泵关闭完成
            pump_result = pump_off_future.result(timeout=1.0)
            if not pump_result:
                self.get_logger().warn("关闭吸泵可能未完成，但继续执行")

            # 5. 执行翻牌动作
            slot_result = self.thread_pool.submit(
                self.controller_app.slot_machine_control, True
            )

            # 6. 移动到中间位置
            if not self.move_to_predefined_position(self.mid_up_pos, "mid_up_pos"):
                return False

            # 7. 并行：拍照和启动吸泵
            parallel_tasks = [
                ("take_picture1", self.take_picture, ()),
                ("start_pump", self.controller_app.pump_control, (1, True)),
            ]
            parallel_results = self.execute_truly_parallel(parallel_tasks)

            # 8. 翻牌复位
            self.controller_app.slot_machine_control(False)

            # 9. 第二次拍照（快速）
            self.take_picture()

            time.sleep(0.5)

            # 10. 移动到翻牌位置
            if not self.move_to_predefined_position(self.flip_up_pos, "flip_up_pos"):
                return False

            # 11. 调整到吸牌位置
            store_up_future_1 = self.thread_pool.submit(
                self.adjust_end_effector_z_from_predefined,
                self.flip_pickup_pos,
                0.05,
                tolerance_1=0.01,
                tolerance=0.04,
            )

            # store_up_future = self.thread_pool.submit(
            #     self.move_to_predefined_position, self.store_flip_pos, "store_flip_pos"
            # )

            duration = time.time() - start_time
            self.stats["flip_time"] += duration
            self.get_logger().info(f"✓ 翻牌动作完成，耗时: {duration:.2f} 秒")

            if not store_up_future_1.result(timeout=5.0):
                self.get_logger().error("未能调整末端执行器Z轴位置")
                return False
            return True

            # # 等待第二个移动完成
            # if not store_up_future.result(timeout=5.0):
            #     return False
            # return True

        except Exception as e:
            self.get_logger().error(
                f"翻牌过程发生异常: {str(e)}\n{traceback.format_exc()}"
            )
            return False

    def drop_card(self):
        """放置卡牌到存放位置"""
        start_time = time.time()
        self.get_logger().info("开始放置卡牌...")

        try:
            # 并行执行：移动和准备关闭吸泵

            store_up_future_1 = self.thread_pool.submit(
                self.move_to_predefined_position,
                self.store_flip_pos,
                "store_flip_pos",
                tolerance=0.045,
            )

            # 同时准备下一个移动
            store_up_future = self.thread_pool.submit(
                self.move_to_predefined_position,
                self.store_up_pos,
                "store_up_pos",
                tolerance=0.04,
            )

            if not store_up_future_1.result(timeout=5.0):
                return False

            # 关闭吸泵释放卡牌
            self.get_logger().info("释放卡牌...")
            # 7. 并行：吸泵
            parallel_tasks = [
                ("off_pump", self.controller_app.pump_control, (1, False)),
            ]

            # 等待第二个移动完成
            if not store_up_future.result(timeout=5.0):
                return False

            parallel_results = self.execute_truly_parallel(parallel_tasks)

            duration = time.time() - start_time
            self.stats["drop_time"] += duration
            self.get_logger().info(f"✓ 卡牌放置成功，耗时: {duration:.2f} 秒")

            return True

        except Exception as e:
            self.get_logger().error(
                f"放置卡牌过程发生异常: {str(e)}\n{traceback.format_exc()}"
            )
            return False

    def run_card_handling_process(self):
        """运行卡牌处理流程"""
        total_start_time = time.time()
        self.get_logger().info("开始卡牌处理流程")
        self.get_logger().info("=" * 50)

        try:
            # 只执行1次循环用于测试
            for cycle in range(1):
                cycle_start_time = time.time()
                self.get_logger().info(f"开始第 {cycle + 1}/1 次循环")
                self.get_logger().info("-" * 30)

                # 取牌
                if not self.pickup_card():
                    self.get_logger().error(f"第 {cycle + 1} 次循环取卡失败")
                    break

                # 翻牌
                if not self.flip_card():
                    self.get_logger().error(f"第 {cycle + 1} 次循环翻牌失败")
                    break

                # 放置
                if not self.drop_card():
                    self.get_logger().error(f"第 {cycle + 1} 次循环放卡失败")
                    break

                cycle_duration = time.time() - cycle_start_time
                self.stats["total_cycles"] += 1
                self.get_logger().info(
                    f"完成第 {cycle + 1} 次循环，耗时: {cycle_duration:.2f} 秒"
                )
                self.get_logger().info("-" * 30)

            total_duration = time.time() - total_start_time
            self.stats["total_time"] = total_duration

            # 输出性能统计
            self.get_logger().info("=" * 50)
            self.get_logger().info("性能统计:")
            self.get_logger().info(f"总循环次数: {self.stats['total_cycles']}")
            self.get_logger().info(f"总耗时: {self.stats['total_time']:.2f} 秒")
            self.get_logger().info(
                f"平均取牌时间: {self.stats['pickup_time']/max(1, self.stats['total_cycles']):.2f} 秒"
            )
            self.get_logger().info(
                f"平均翻牌时间: {self.stats['flip_time']/max(1, self.stats['total_cycles']):.2f} 秒"
            )
            self.get_logger().info(
                f"平均放置时间: {self.stats['drop_time']/max(1, self.stats['total_cycles']):.2f} 秒"
            )
            self.get_logger().info(f"并行操作次数: {self.stats['parallel_ops']}")
            self.get_logger().info(
                f"并行节省时间: {self.stats['parallel_saved_time']:.2f} 秒"
            )
            self.get_logger().info("=" * 50)

        except KeyboardInterrupt:
            self.get_logger().info("用户中断卡牌处理流程")
        except Exception as e:
            self.get_logger().error(
                f"卡牌处理流程发生异常: {str(e)}\n{traceback.format_exc()}"
            )
        finally:
            self.thread_pool.shutdown(wait=True)
            self.get_logger().info("卡牌处理流程结束")


def main(args=None):
    rclpy.init(args=args)
    node = None  # 初始化为None，确保在异常处理中可以访问

    try:
        node = ArmCardDealerNode()
        node.get_logger().info("ArmCardDealerNode 已启动.")

        # 使用线程运行ROS消息处理
        spin_thread = Thread(target=lambda n=node: rclpy.spin(n), args=(node,))
        spin_thread.daemon = True
        spin_thread.start()

        # 等待接收位置数据
        timeout = 3.0  # 3秒超时
        wait_start = time.time()

        node.get_logger().info("等待接收末端执行器位置数据...")

        while node.current_pose is None and (time.time() - wait_start) < timeout:
            time.sleep(0.05)
            if node.pose_ready_event.wait(0.05):
                break

        if node.current_pose is None:
            node.get_logger().warn(
                f"在 {timeout} 秒内未能接收末端执行器位置数据，继续执行..."
            )
        else:
            node.get_logger().info(
                f"成功接收末端执行器位置数据，等待时间: {time.time() - wait_start:.2f} 秒"
            )

        # 运行卡牌处理流程
        node.run_card_handling_process()

        if node:
            node.destroy_node()

    except KeyboardInterrupt:
        if node:
            node.get_logger().info("程序被用户中断")
    except Exception as e:
        if node:
            node.get_logger().error(f"程序发生异常: {str(e)}\n{traceback.format_exc()}")
        else:
            print(f"程序发生异常: {str(e)}\n{traceback.format_exc()}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
