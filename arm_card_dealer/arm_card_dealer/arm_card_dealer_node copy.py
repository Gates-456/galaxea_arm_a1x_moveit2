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
        # 定义一个模拟类用于调试
        class ControllerApp:
            def __init__(self):
                print("⚠️ 注意：未能找到真实的ControllerApp，使用模拟版本")

            def get_tof_data(self):
                print("模拟TOF数据：80mm")
                return 80  # 模拟返回80mm

            def pump_control(self, id, state):
                print(f"模拟泵控制：ID {id}, 状态 {state}")
                return True

            def slot_machine_control(self, state):
                print(f"模拟翻牌机控制：状态 {state}")
                return True


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

        # 预定义位置
        self.initial_position = [0.0, 0.0, 0.3]  # 初始化位置
        self.pickup_approach_position = [0.2, 0.0, 0.3]  # 接近取牌位置
        self.pickup_position = [0.2, 0.0, 0.15]  # 取牌位置
        self.flip_position = [0.0, 0.2, 0.3]  # 翻牌位置
        self.drop_position = [-0.2, 0.0, 0.3]  # 存放位置

        # arm 初始化 0的位置
        # position:
        #     x: -0.02279700000000001
        #     y: 0.0
        #     z: 0.210785
        # orientation:
        #     x: 0.0
        #     y: 0.0
        #     z: 0.0
        #     w: 1.0

        # arm 初始化 立起来的位置
        # position:
        #     x: 0.15682272765101235
        #     y: -5.699999999914107e-07
        #     z: 0.32770900338005976
        # orientation:
        #     x: 0.0
        #     y: 0.6830386157896124
        #     z: 0.0
        #     w: 0.7303822624764448

        # arm取牌盒子顶部位置
        # position:
        #     x: 0.16034231095016882
        #     y: -0.0012287479066121036
        #     z: 0.13703169136011317
        # orientation:
        #     x: 0.00437788368298138
        #     y: 0.7173426206091026
        #     z: -0.004347129296294568
        #     w: 0.696693261958952

        # arm取牌盒子顶部 牌安全位置
        # position:
        #     x: 0.16201403777255452
        #     y: -0.0012415528731179806
        #     z: 0.17056220945809003
        # orientation:
        #     x: 0.004225984924417524
        #     y: 0.7151163504154242
        #     z: -0.004198518865056019
        #     w: 0.6989800561241232

        # arm 取牌和翻牌中间下位置
        # position:
        #     x: 0.1621868472479211
        #     y: 0.04380610579531642
        #     z: 0.15712776892968258
        # orientation:
        #     x: 0.004776064377606067
        #     y: 0.71518405442191
        #     z: -0.00474986515781664
        #     w: 0.6989037103140044

        # arm 取牌和翻牌中间上位置
        # position:
        #     x: 0.16276085361633363
        #     y: 0.04281107543389526
        #     z: 0.18722631464049
        # orientation:
        #     x: 0.004444271270389836
        #     y: 0.7146842269576513
        #     z: -0.004025430096679523
        #     w: 0.6994215467823081

        # arm 翻牌上位置
        # position:
        #     x: 0.16442152820072622
        #     y: 0.17728722101303435
        #     z: 0.18713015403913438
        # orientation:
        #     x: 0.00431649045219734
        #     y: 0.7148960134030288
        #     z: -0.004044064243847345
        #     w: 0.6992057661912009

        # arm 翻牌 放牌位置
        # position:
        #     x: 0.16330703786518166
        #     y: 0.17700603728634517
        #     z: 0.1317986813141175
        # orientation:
        #     x: 0.005155227344106672
        #     y: 0.7146524052453089
        #     z: -0.003950793248584494
        #     w: 0.699449608292715

        # arm 翻牌 吸牌位置
        # position:
        #     x: 0.16304883813417456
        #     y: 0.17701450715624056
        #     z: 0.11928420469847326
        # orientation:
        #     x: 0.00502690383288279
        #     y: 0.7144932773609015
        #     z: -0.0039254613176109565
        #     w: 0.6996132342926175

        # arm 存牌上位置
        # position:
        #     x: 0.22929217209135816
        #     y: -8.856539774223389e-05
        #     z: 0.14564830615132662
        # orientation:
        #     x: 0.004320177785579656
        #     y: 0.7150426807208595
        #     z: -0.004107737182698441
        #     w: 0.6990553821455534

        # 卡牌相关参数
        self.card_height = 0.01  # 单张卡牌厚度(m)
        self.pickup_safe_height = 0.02  # 吸盘吸取高度补偿(m)
        self.pickup_approach_height = 0.05  # 接近卡牌的高度(m)

        self.get_logger().info(
            f"ArmCardDealerNode initialized with group: {self.move_group_name}, "
            f"end effector: {self.end_effector_link}"
        )

    def send_initial_position(self):
        """发送初始化位置"""
        self.get_logger().info("Moving to initial position...")
        self.move_to_position(
            self.initial_position[0], self.initial_position[1], self.initial_position[2]
        )
        time.sleep(2)  # 等待到达

    def get_tof_and_calculate_height(self):
        """获取TOF传感器数据并计算卡牌高度"""
        tof_data = self.controller_app.get_tof_data()
        if tof_data is not None:
            # 假设TOF传感器距离取牌位置的高度固定，减去TOF读数得到卡牌表面高度
            # 这里需要根据实际安装情况调整计算公式
            sensor_offset = 0.2  # 假设传感器离最低点的距离为0.2m
            card_surface_height = sensor_offset - (tof_data / 1000.0)  # 转换为米

            self.get_logger().info(
                f"TOF data: {tof_data}mm, calculated surface height: {card_surface_height:.3f}m"
            )

            # 如果检测到有牌（距离小于某个阈值，比如10cm）
            if tof_data > 0 and tof_data < 100:  # 假设10cm以内表示有牌
                return True, card_surface_height
            else:
                self.get_logger().info("No cards detected")
                return False, 0.0
        else:
            self.get_logger().error("Failed to get TOF data")
            return False, 0.0

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

    def move_to_position(self, x, y, z):
        """移动到指定位置"""
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        # 默认方向朝下（z轴负方向）
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.target_pose_publisher.publish(pose_msg)
        self.get_logger().info(f"Published target pose: ({x:.3f}, {y:.3f}, {z:.3f})")

    def wait_for_arrival(self, target_x, target_y, target_z, tolerance=0.01):
        """等待机械臂到达目标位置"""
        self.get_logger().info(
            f"Waiting to arrive at position: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})"
        )

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

    def pickup_card(self, card_surface_height):
        """执行取卡动作"""
        self.get_logger().info("Starting card pickup process...")

        # 1. 移动到取牌位置上方
        self.move_to_position(
            self.pickup_approach_position[0],
            self.pickup_approach_position[1],
            self.pickup_approach_position[2],
        )

        if not self.wait_for_arrival(
            self.pickup_approach_position[0],
            self.pickup_approach_position[1],
            self.pickup_approach_position[2],
        ):
            self.get_logger().error("Failed to reach pickup approach position")
            return False

        # 2. 缓慢下降到接近卡牌的位置
        approach_z = card_surface_height + self.pickup_approach_height
        self.move_to_position(
            self.pickup_position[0], self.pickup_position[1], approach_z
        )

        if not self.wait_for_arrival(
            self.pickup_position[0], self.pickup_position[1], approach_z
        ):
            self.get_logger().error("Failed to reach card approach position")
            return False

        # 3. 启动吸泵
        self.get_logger().info("Turning on suction pump...")
        if not self.controller_app.pump_control(1, True):
            self.get_logger().error("Failed to turn on suction pump")
            return False

        # 4. 精确下降到卡牌表面
        pickup_z = card_surface_height + self.pickup_safe_height
        self.move_to_position(
            self.pickup_position[0], self.pickup_position[1], pickup_z
        )

        if not self.wait_for_arrival(
            self.pickup_position[0], self.pickup_position[1], pickup_z
        ):
            self.get_logger().error("Failed to reach pickup position")
            return False

        time.sleep(0.5)  # 等待吸盘吸住卡牌

        # 5. 上升离开取牌区
        self.move_to_position(
            self.pickup_position[0],
            self.pickup_position[1],
            self.pickup_approach_position[2],
        )

        if not self.wait_for_arrival(
            self.pickup_position[0],
            self.pickup_position[1],
            self.pickup_approach_position[2],
        ):
            self.get_logger().error("Failed to rise from pickup position")
            return False

        self.get_logger().info("Card pickup completed successfully")
        return True

    def flip_card(self):
        """执行翻牌动作"""
        self.get_logger().info("Moving to flip position...")

        # 移动到翻牌位置
        self.move_to_position(
            self.flip_position[0], self.flip_position[1], self.flip_position[2]
        )

        if not self.wait_for_arrival(
            self.flip_position[0], self.flip_position[1], self.flip_position[2]
        ):
            self.get_logger().error("Failed to reach flip position")
            return False

        # 执行翻牌动作
        self.get_logger().info("Flipping card...")
        self.controller_app.slot_machine_control(True)
        time.sleep(0.2)
        self.controller_app.slot_machine_control(False)

        # 拍照（这里只是模拟，实际需要集成摄像头）
        self.get_logger().info("Taking photo after flipping card...")
        time.sleep(1)  # 模拟拍照时间

        self.get_logger().info("Card flip completed successfully")
        return True

    def drop_card(self):
        """放置卡牌到存放位置"""
        self.get_logger().info("Moving to drop position...")

        # 移动到存放位置
        self.move_to_position(
            self.drop_position[0], self.drop_position[1], self.drop_position[2]
        )

        if not self.wait_for_arrival(
            self.drop_position[0], self.drop_position[1], self.drop_position[2]
        ):
            self.get_logger().error("Failed to reach drop position")
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

        # 1. 检查是否有卡牌
        tof_data = self.controller_app.get_tof_data()
        if tof_data is not None:
            self.get_logger().info(f"TOF距离: {tof_data} mm")

            # 如果tof_data大于125mm或小于60mm，则认为无牌
            if tof_data > 125 or tof_data < 60:
                self.get_logger().info(f"TOF距离:{tof_data},超出范围，无牌")
            else:
                self.get_logger().info(f"TOF距离:{tof_data},在范围内，有牌")

                # 计算机器人高度
                pos_z = self.calculate_robot_height(tof_data)
                self.get_logger().info(f"计算得到的机械臂高度: {pos_z:.3f} m")
        else:
            self.get_logger().error("无法获取TOF距离数据")
        return 0

        # 发送初始化位置
        self.send_initial_position()

        # 循环执行10次或直到没有卡牌
        for cycle in range(10):
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
            # 获取TOF数据并计算卡牌高度
            has_cards, card_height = self.get_tof_and_calculate_height()
            if not has_cards:
                self.get_logger().info("No more cards detected, stopping process")
                break

            if not self.pickup_card(card_height):
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
        self.send_initial_position()
        self.get_logger().info("Card handling process completed")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ArmCardDealerNode()
        node.get_logger().info("ArmCardDealerNode started.")

        # 等待一些时间让系统初始化
        node.get_logger().info("Waiting for systems to initialize...")
        time.sleep(2)

        # 运行卡牌处理流程
        node.run_card_handling_process()

        node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
