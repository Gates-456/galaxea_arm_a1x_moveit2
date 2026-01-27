#!/usr/bin/env python3
"""
A1X 机械臂硬件接口插件
功能：从真实机械臂获取状态并发布到 joint_states 话题
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from controller_manager import hardware_interface
from controller_manager.hardware_interface import HardwareInterface
from controller_manager.state_interfaces import StateInterface
from controller_manager.command_interfaces import CommandInterface
import threading
import time


class A1XHardwareInterface(HardwareInterface):
    """A1X 机械臂硬件接口"""

    def __init__(self):
        super().__init__()
        self.joint_names = [
            "arm_joint1",
            "arm_joint2",
            "arm_joint3",
            "arm_joint4",
            "arm_joint5",
            "arm_joint6",
        ]

        # 关节状态
        self.joint_positions = [0.0] * 6
        self.joint_velocities = [0.0] * 6
        self.joint_efforts = [0.0] * 6

        # 关节命令
        self.joint_commands = [0.0] * 6

        # ROS2 节点
        self.node = None
        self.joint_state_sub = None
        self.state_lock = threading.Lock()

        # 硬件接口状态
        self.hardware_state = "initialized"

    def init(self, info, hardware_params):
        """初始化硬件接口"""
        # 创建 ROS2 节点
        rclpy.init()
        self.node = Node("a1x_hardware_interface")

        # 订阅真实机械臂的关节状态
        self.joint_state_sub = self.node.create_subscription(
            JointState, "/hdas/feedback_arm", self.joint_state_callback, 10
        )

        self.node.get_logger().info("A1X 硬件接口初始化完成")
        return True

    def joint_state_callback(self, msg):
        """处理真实机械臂的关节状态消息"""
        with self.state_lock:
            # 真实机械臂返回的数据结构：
            # name: ["arm"]
            # position: [0.0, -0.001276595744680851, 0.0, 0.0, 0.0, 0.0, 0.0]
            # velocity: [-0.004, 0.006666666666666667, -0.004, -0.017333333333333333, -0.009333333333333334, -0.0026666666666666666, 0.0]
            # effort: [0.58, 3.1366666666666667, -3.37, -0.6183333333333333, 0.16, 0.03666666666666667, 0.0]
            #
            # 注意：真实机械臂返回的数据包含7个值，第一个值似乎是某种状态或标识
            # 我们需要将其映射到我们的6个关节 0-6 7是夹爪

            if len(msg.position) >= 7:
                # 跳过第一个值，从第二个值开始
                self.joint_positions = list(msg.position[0:6])

            if len(msg.velocity) >= 7:
                # 跳过第一个值，从第二个值开始
                self.joint_velocities = list(msg.velocity[0:6])
                # 打印六个关节的最大速度值
                max_velocity = max(self.joint_velocities)
                self.node.get_logger().info(f'六个关节的最大速度值: {max_velocity}')

            if len(msg.effort) >= 7:
                # 跳过第一个值，从第二个值开始
                self.joint_efforts = list(msg.effort[0:6])

    def read(self, time, period):
        """读取硬件状态"""
        # 旋转回调以处理订阅的消息
        rclpy.spin_once(self.node, timeout_sec=0.001)
        return hardware_interface.return_type.OK

    def write(self, time, period):
        """写入硬件命令"""
        # 这里可以将关节命令发送到真实机械臂
        # 目前只是存储命令
        return hardware_interface.return_type.OK

    def export_state_interfaces(self):
        """导出状态接口"""
        state_interfaces = []
        for i, joint_name in enumerate(self.joint_names):
            state_interfaces.append(
                StateInterface(
                    joint_name, "position", lambda idx=i: self.joint_positions[idx]
                )
            )
            state_interfaces.append(
                StateInterface(
                    joint_name, "velocity", lambda idx=i: self.joint_velocities[idx]
                )
            )
        return state_interfaces

    def export_command_interfaces(self):
        """导出命令接口"""
        command_interfaces = []
        for i, joint_name in enumerate(self.joint_names):
            command_interfaces.append(
                CommandInterface(
                    joint_name,
                    "position",
                    lambda value, idx=i: self._set_joint_command(idx, value),
                )
            )
        return command_interfaces

    def _set_joint_command(self, joint_idx, value):
        """设置关节命令"""
        self.joint_commands[joint_idx] = value
