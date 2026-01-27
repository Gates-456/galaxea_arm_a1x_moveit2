#!/usr/bin/env python3

"""
卡牌处理节点，实现取牌、放置、翻牌等功能
"""

import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from math import sqrt
import sys
import os

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
        self.pickup_top_pos = [0.1603, -0.0012, 0.1370, 0.0044, 0.7173, -0.0043, 0.6967]

        # arm取牌 盒子顶部 牌安全位置
        self.pickup_safe_pos = [
            0.1620,
            -0.0012,
            0.1706,
            0.0042,
            0.7151,
            -0.0042,
            0.6990,
        ]

        # arm 取牌和翻牌中间下位置
        self.mid_down_pos = [0.1622, 0.0438, 0.1571, 0.0048, 0.7152, -0.0047, 0.6989]

        # arm 取牌和翻牌中间上位置
        self.mid_up_pos = [0.1628, 0.0428, 0.1872, 0.0044, 0.7147, -0.0040, 0.6994]

        # arm 翻牌上位置
        self.flip_up_pos = [0.1644, 0.1773, 0.1871, 0.0043, 0.7149, -0.0040, 0.6992]

        # arm 翻牌 放牌位置
        self.flip_drop_pos = [0.1633, 0.1770, 0.1318, 0.0052, 0.7147, -0.0040, 0.6994]

        # arm 翻牌 吸牌位置
        self.flip_pickup_pos = [0.1630, 0.1770, 0.1193, 0.0050, 0.7145, -0.0039, 0.6996]

        # arm 存牌上位置
        self.store_up_pos = [0.2293, -0.0001, 0.1456, 0.0043, 0.7150, -0.0041, 0.6991]

        self.get_logger().info(
            f"ArmCardDealerNode initialized with group: {self.move_group_name}, "
            f"end effector: {self.end_effector_link}"
        )

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
            self.get_logger().error("Laser measurement cannot be None")
            raise ValueError("Laser measurement cannot be None")

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

    def move_to_predefined_position(self, position_var, position_name="predefined"):
        """
        移动到预定义位置

        Args:
            position_var: 预定义位置变量，包含[x, y, z, ox, oy, oz, ow]的列表
            position_name: 位置名称，用于日志记录
        """
        if len(position_var) != 7:
            self.get_logger().error(
                f"Invalid position variable for {position_name}: expected 7 values [x,y,z,ox,oy,oz,ow]"
            )
            return False

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
            f"Published predefined pose {position_name}: "
            f"Position({x:.4f}, {y:.4f}, {z:.4f}), "
            f"Orientation({ox:.4f}, {oy:.4f}, {oz:.4f}, {ow:.4f})"
        )

        # 等待到达位置，允许1mm误差
        return self.wait_for_arrival(x, y, z, tolerance=0.001)

    def wait_for_arrival(self, target_x, target_y, target_z, tolerance=0.01):
        """等待机械臂到达目标位置"""
        self.get_logger().info(
            f"Waiting to arrive at position: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})"
        )
        return True

        start_time = time.time()
        timeout = 30  # 30秒超时

        while time.time() - start_time < timeout:
            current_pose = self.get_current_pose()
            if current_pose is not None:
                distance = sqrt(
                    (current_pose.position.x - target_x) ** 2
                    + (current_pose.position.y - target_y) ** 2
                    + (current_pose.position.z - target_z) ** 2
                )

                if distance <= tolerance:
                    self.get_logger().info(
                        f"Arrived at target position, distance: {distance:.3f}"
                    )
                    return True

            time.sleep(0.1)  # 每0.1秒检查一次

        self.get_logger().warn(
            f"Timeout waiting for arrival at ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})"
        )
        return False

    def adjust_end_effector_z_from_predefined(
        self, position_var, z_adjustment, step_size=0.005
    ):
        """
        从预定义位置开始，对末端执行器Z轴进行微调

        Args:
            position_var: 预定义位置变量，包含[x, y, z, ox, oy, oz, ow]的列表
            z_adjustment: Z轴调整量（米），正值为上升，负值为下降
            step_size: 每步移动距离（米），默认0.001m (1mm)
        """
        if len(position_var) != 7:
            self.get_logger().error(
                "Invalid position variable: expected 7 values [x,y,z,ox,oy,oz,ow]"
            )
            return False

        # 先移动到预定义位置
        success = self.move_to_predefined_position(position_var, "base_position")
        if not success:
            self.get_logger().error("Failed to move to predefined position")
            return False

        # 等待到达预定义位置
        x, y, z, ox, oy, oz, ow = position_var
        if not self.wait_for_arrival(x, y, z, tolerance=0.001):
            self.get_logger().error("Failed to reach predefined position")
            return False

        # 获取当前位置作为基准
        current_pose = self.get_current_pose()
        if current_pose is None:
            self.get_logger().error(
                "Failed to get current pose after moving to predefined position"
            )
            return False

        # 计算最终Z轴位置
        target_z = current_pose.position.z + z_adjustment

        # 计算移动方向
        direction = "up" if z_adjustment > 0 else "down"

        # 计算需要多少步
        num_steps = int(abs(z_adjustment) / step_size)

        if num_steps == 0:
            self.get_logger().info(
                f"Z adjustment too small: {z_adjustment}m, setting directly"
            )
            # 直接移动到目标位置，使用publisher发布位姿
            pose_msg = PoseStamped()
            pose_msg.header = Header()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "base_link"
            pose_msg.pose.position.x = current_pose.position.x
            pose_msg.pose.position.y = current_pose.position.y
            pose_msg.pose.position.z = target_z
            pose_msg.pose.orientation = current_pose.orientation

            self.target_pose_publisher.publish(pose_msg)
            self.get_logger().info(
                f"Published direct target pose: ({current_pose.position.x:.3f}, {current_pose.position.y:.3f}, {target_z:.3f})"
            )

            return self.wait_for_arrival(
                current_pose.position.x,
                current_pose.position.y,
                target_z,
                tolerance=0.001,
            )

        # 计算每步的实际移动距离
        actual_step = abs(z_adjustment) / num_steps
        step_multiplier = 1 if z_adjustment > 0 else -1

        self.get_logger().info(
            f"Starting Z-axis adjustment: {z_adjustment}m in {num_steps} steps of {actual_step:.5f}m"
        )

        for i in range(1, num_steps + 1):
            # 计算当前位置
            new_z = current_pose.position.z + step_multiplier * actual_step * i

            # 直接使用publisher发布位姿，而不是move_to_position
            pose_msg = PoseStamped()
            pose_msg.header = Header()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "base_link"
            pose_msg.pose.position.x = current_pose.position.x
            pose_msg.pose.position.y = current_pose.position.y
            pose_msg.pose.position.z = new_z
            pose_msg.pose.orientation = current_pose.orientation

            self.target_pose_publisher.publish(pose_msg)

            # 每10步输出一次进度
            if i % 10 == 0 or i == num_steps:
                progress_percent = (i / num_steps) * 100
                self.get_logger().info(
                    f"Z-axis adjustment progress: {progress_percent:.1f}% ({i}/{num_steps})"
                )

            # 等待到达当前位置
            if not self.wait_for_arrival(
                current_pose.position.x, current_pose.position.y, new_z, tolerance=0.001
            ):
                self.get_logger().warn(
                    f"Failed to reach intermediate position at step {i}"
                )
                # 继续执行，不终止整个过程

            # 短暂延时，让移动执行
            time.sleep(0.01)  # 10ms的小延时，让系统有时间处理移动命令

        # 最后确保到达目标位置
        final_success = self.wait_for_arrival(
            current_pose.position.x, current_pose.position.y, target_z, tolerance=0.001
        )
        if final_success:
            self.get_logger().info(
                f"Z-axis adjustment completed: {z_adjustment}m from predefined position"
            )
        else:
            self.get_logger().warn(
                f"Z-axis adjustment finished but could not confirm arrival at target: {z_adjustment}m"
            )

        return final_success

    def send_initial_position(self):
        """发送初始化位置"""
        self.get_logger().info("Moving to initial position...")
        if self.move_to_predefined_position(
            self.initial_upright_pos, "initial_upright_pos"
        ):
            return True
        else:
            return False

    def pickup_card(self, card_surface_height):
        """执行取卡动作"""
        self.get_logger().info("Starting card pickup process...")

        # arm取牌 盒子顶部 牌安全位置
        if not self.move_to_predefined_position(
            self.pickup_safe_pos, "pickup_safe_pos"
        ):
            return False

        # 启动吸泵
        self.get_logger().info("Turning on suction pump...")
        if not self.controller_app.pump_control(1, True):
            self.get_logger().error("Failed to turn on suction pump")
            return False

        # arm取牌 盒子顶部位置
        if not self.adjust_end_effector_z_from_predefined(
            self.pickup_top_pos,
            card_surface_height,
        ):
            self.get_logger().error("not adjust_end_effector_z_from_predefined")
            return False

        # arm取牌 盒子顶部 牌安全位置
        if not self.move_to_predefined_position(
            self.pickup_safe_pos, "pickup_safe_pos"
        ):
            return False

        # arm 取牌和翻牌中间下位置
        if not self.move_to_predefined_position(self.mid_down_pos, "mid_down_pos"):
            return False

        # arm 取牌和翻牌中间上位置
        if not self.move_to_predefined_position(self.mid_up_pos, "mid_up_pos"):
            return False

        return True

    # 拍照
    def take_picture(self):
        # 拍照（这里只是模拟，实际需要集成摄像头）
        self.get_logger().info("Taking photo after flipping card...")
        time.sleep(0.3)  # 模拟拍照时间
        return True

    def flip_card(self):
        """执行翻牌动作"""
        self.get_logger().info("Moving to flip position...")

        # 移动到翻牌位置
        # arm 翻牌上位置
        if not self.move_to_predefined_position(self.flip_up_pos, "flip_up_pos"):
            return False

        # arm 翻牌 放牌位置
        if not self.move_to_predefined_position(self.flip_up_pos, "flip_up_pos"):
            return False

        self.get_logger().info("Turning off suction pump...")
        if not self.controller_app.pump_control(1, False):
            self.get_logger().error("Failed to turn off suction pump")
            return False

        # arm 取牌和翻牌中间上位置
        if not self.move_to_predefined_position(self.mid_up_pos, "mid_up_pos"):
            return False

        if not self.take_picture():
            return False

        # 执行翻牌动作
        self.get_logger().info("Flipping card...")
        self.controller_app.slot_machine_control(True)
        time.sleep(0.2)
        self.controller_app.slot_machine_control(False)

        if not self.take_picture():
            return False

        # 启动吸泵
        self.get_logger().info("Turning on suction pump...")
        if not self.controller_app.pump_control(1, True):
            self.get_logger().error("Failed to turn on suction pump")
            return False

        # arm 吸牌位置
        if not self.move_to_predefined_position(
            self.flip_pickup_pos, "flip_pickup_pos"
        ):
            return False

        # arm 翻牌上位置
        if not self.move_to_predefined_position(self.flip_up_pos, "flip_up_pos"):
            return False

        self.get_logger().info("Card flip completed successfully")
        return True

    def drop_card(self):
        """放置卡牌到存放位置"""
        self.get_logger().info("Moving to drop position...")

        # arm 存牌上位置
        if not self.move_to_predefined_position(self.store_up_pos, "store_up_pos"):
            return False

        # 关闭吸泵释放卡牌
        self.get_logger().info("Releasing card...")
        if not self.controller_app.pump_control(1, False):
            self.get_logger().error("Failed to turn off suction pump")
            return False

        self.get_logger().info("Card dropped successfully")
        return True

    def get_current_pose(self):
        """获取当前末端执行器的姿态"""
        try:
            # 使用TF获取末端执行器相对于基座的变换
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                self.end_effector_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )

            # 将变换转换为Pose
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation

            return pose

        except TransformException as ex:
            # 降低日志级别，避免频繁警告
            self.get_logger().debug(f"Could not get transform: {ex}")
            return None
        except Exception as e:
            self.get_logger().error(f"Error getting current pose: {str(e)}")
            return None

    def run_card_handling_process(self):
        """运行完整的卡牌处理流程"""
        self.get_logger().info("Starting card handling process...")

        # 发送初始化位置
        if not self.send_initial_position():
            self.get_logger().error(f"no send_initial_position")
            return False

        # 循环执行10次或直到没有卡牌
        for cycle in range(1):
            self.get_logger().info(f"Starting cycle {cycle + 1}/10")

            # 1. 检查是否有卡牌
            tof_data = self.controller_app.get_tof_data()
            if tof_data is not None:
                self.get_logger().info(f"TOF距离: {tof_data} mm")

                # 如果tof_data大于125mm或小于60mm，则认为无牌
                if tof_data > 125 or tof_data < 60:
                    self.get_logger().info(f"TOF距离:{tof_data},超出范围，无牌")
                    break  # 没有牌了，退出循环
                else:
                    self.get_logger().info(f"TOF距离:{tof_data},在范围内，有牌")

                    # 计算机器人高度
                    pos_z = self.calculate_robot_height(tof_data)
                    self.get_logger().info(f"计算得到的机械臂高度: {pos_z:.3f} m")
            else:
                self.get_logger().error("无法获取TOF距离数据")
                break

            # 2. 开始取牌
            if not self.pickup_card(pos_z):
                self.get_logger().error(f"Failed to pickup card in cycle {cycle + 1}")
                continue

            # 3. 移动到翻牌位置并翻牌
            if not self.flip_card():
                self.get_logger().error(f"Failed to flip card in cycle {cycle + 1}")
                continue

            # 4. 放置卡牌到存放位置
            if not self.drop_card():
                self.get_logger().error(f"Failed to drop card in cycle {cycle + 1}")
                continue

            self.get_logger().info(f"Completed cycle {cycle + 1}")

        # 回到初始位置
        if not self.send_initial_position():
            self.get_logger().error(f"no send_initial_position")
            return False
        self.get_logger().info("Card handling process completed")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ArmCardDealerNode()
        node.get_logger().info("ArmCardDealerNode started.")

        # 等待一些时间让系统初始化
        node.get_logger().info("Waiting for systems to initialize...")

        # 运行卡牌处理流程
        node.run_card_handling_process()

        node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
