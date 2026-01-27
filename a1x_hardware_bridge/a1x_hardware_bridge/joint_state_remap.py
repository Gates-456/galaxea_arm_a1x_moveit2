#!/usr/bin/env python3
"""
A1X 机械臂关节状态重映射节点
功能：从真实机械臂获取状态并发布到 joint_states 话题
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateRemap(Node):
    """关节状态重映射节点"""

    def __init__(self):
        super().__init__("joint_state_remap")

        # 关节名称映射
        self.joint_names = [
            "arm_joint1",
            "arm_joint2",
            "arm_joint3",
            "arm_joint4",
            "arm_joint5",
            "arm_joint6",
        ]

        # 订阅真实机械臂的关节状态
        self.arm_state_sub = self.create_subscription(
            JointState,
            "/hdas/feedback_arm",
            self.arm_state_callback,
            10
        )

        # 发布到标准的 joint_states 话题
        self.joint_state_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10
        )

        self.get_logger().info("关节状态重映射节点启动")
        self.get_logger().info(f"订阅: /hdas/feedback_arm")
        self.get_logger().info(f"发布: /joint_states")

    def arm_state_callback(self, msg):
        """处理真实机械臂的关节状态消息"""
        # 创建新的 joint_states 消息
        joint_state_msg = JointState()
        joint_state_msg.header = msg.header
        joint_state_msg.name = self.joint_names

        # 真实机械臂返回的数据结构：
        # position[关节1电机位置反馈, 关节2电机位置反馈, 关节3电机位置反馈, 
        #          关节4电机位置反馈, 关节5电机位置反馈, 关节6电机位置反馈, 
        #          夹爪电机位置反馈]
        # velocity[关节1电机速度反馈, 关节2电机速度反馈, 关节3电机速度反馈, 
        #          关节4电机速度反馈, 关节5电机速度反馈, 关节6电机速度反馈，夹爪电机速度反馈]
        # effort[关节1电机力矩反馈, 关节2电机力矩反馈, 关节3电机力矩反馈, 
        #        关节4电机力矩反馈, 关节5电机力矩反馈, 关节6电机力矩反馈, 夹爪电机力矩反馈]
        #
        # 我们只需要前6个关节的数据（不包括夹爪）

        # 添加调试信息
        self.get_logger().debug(f"接收到真实机械臂状态: name={msg.name}, position={msg.position}")

        if len(msg.position) >= 6:
            # 只取前6个关节的位置
            joint_state_msg.position = list(msg.position[0:6])
            self.get_logger().debug(f"映射后的位置: {joint_state_msg.position}")

        if len(msg.velocity) >= 6:
            # 只取前6个关节的速度
            joint_state_msg.velocity = list(msg.velocity[0:6])

        if len(msg.effort) >= 6:
            # 只取前6个关节的力矩
            joint_state_msg.effort = list(msg.effort[0:6])

        # 发布到 joint_states 话题
        self.joint_state_pub.publish(joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRemap()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被用户终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
