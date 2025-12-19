#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# 导入 A1X 特有的消息类型
try:
    from hdas_msg.msg import MotorControl
except ImportError:
    pass  # 如果没有找到 hdas_msg，我们将只使用标准的 JointState


class A1XBridge(Node):
    def __init__(self):
        super().__init__("a1x_bridge_node")

        # --- 配置 ---
        # 我们定义机械臂的关节名称，必须与 URDF 一致
        self.arm_joint_names = [
            "arm_joint1",
            "arm_joint2",
            "arm_joint3",
            "arm_joint4",
            "arm_joint5",
            "arm_joint6",
        ]

        # --- 1. 反馈链路 (Feedback): Robot -> ROS ---
        # 订阅真实机械臂反馈
        self.robot_feedback_sub = self.create_subscription(
            JointState, "/hdas/feedback_arm", self.robot_feedback_callback, 10
        )

        # 发布给 ROS 全局，驱动 RViz 模型
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)

        # --- 2. 控制链路 (Command): MoveIt -> Robot ---
        # 订阅 MoveIt 控制器的输出。注意：这里我们订阅控制器发出的指令
        # 通常 MoveIt 会通过控制器发布到这个话题
        self.moveit_cmd_sub = self.create_subscription(
            JointState,
            "/a1x_arm_controller/joint_states",  # 对应之前配置的控制器输出
            self.moveit_cmd_callback,
            10,
        )

        # 发布给真实机械臂的控制接口
        self.robot_cmd_pub = self.create_publisher(
            JointState, "/motion_target/target_joint_state_arm", 10
        )

        self.get_logger().info("A1X 硬件桥接节点启动成功！")
        self.get_logger().info("正在监听反馈: /hdas/feedback_arm")
        self.get_logger().info("正在转发控制: /motion_target/target_joint_state_arm")

    def robot_feedback_callback(self, msg):
        """将机械臂反馈转发到全局 /joint_states，让 RViz 同步动作"""
        # A1X 反馈的消息里可能包含 7 个值（包含夹爪），我们直接转发即可
        # 确保时间戳是最新的
        msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_pub.publish(msg)

    def moveit_cmd_callback(self, msg):
        """将 MoveIt 的规划结果转发给机械臂"""
        # 创建一个符合 A1X 接口要求的 JointState 消息
        target_msg = JointState()
        target_msg.header.stamp = self.get_clock().now().to_msg()

        # A1X 关节控制接口要求提供 6 个关节的位置
        # 我们根据名称匹配，确保顺序正确
        try:
            positions = []
            for name in self.arm_joint_names:
                if name in msg.name:
                    idx = msg.name.index(name)
                    positions.append(msg.position[idx])

            if len(positions) == 6:
                target_msg.name = self.arm_joint_names
                target_msg.position = positions
                # 根据文档，加速度和速度限制由底层处理，这里我们只发位置
                self.robot_cmd_pub.publish(target_msg)
        except Exception as e:
            self.get_logger().error(f"转发控制指令失败: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = A1XBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
