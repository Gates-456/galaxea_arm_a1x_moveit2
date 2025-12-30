#!/usr/bin/env python3
"""
A1X 机械臂 MoveIt2 执行桥接
功能：从控制器状态获取轨迹信息，转发至机械臂驱动接口。
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState


class A1XTrajectoryBridge(Node):
    def __init__(self):
        super().__init__("a1x_trajectory_bridge")

        # 关节名 (与MoveIt配置及你的测试保持一致)
        self.joint_names = [
            "arm_joint1",
            "arm_joint2",
            "arm_joint3",
            "arm_joint4",
            "arm_joint5",
            "arm_joint6",
        ]

        # 订阅控制器状态话题，使用正确的控制器状态话题
        self.controller_state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            "/a1x_group_controller/controller_state",
            self.controller_state_callback,
            10,
        )

        # --- 发布到机械臂驱动接口 ---
        self.arm_cmd_pub = self.create_publisher(
            JointState, "/motion_target/target_joint_state_arm", 10
        )

        # 存储上一次发送的关节状态，用于检测变化
        self.last_sent_positions = [0.0] * 6
        self.last_sent_velocities = [0.0] * 6

        # 存储每个关节的最后大于0.01的速度值
        self.last_above_threshold_velocities = [0.4] * 6

        # 添加标志来跟踪是否有活动轨迹
        self.has_active_trajectory = False

        self.get_logger().info("=" * 60)
        self.get_logger().info("A1X 轨迹桥接节点启动")
        self.get_logger().info(f"订阅: {self.controller_state_sub.topic_name}")
        self.get_logger().info(f"发布: {self.arm_cmd_pub.topic_name}")
        self.get_logger().info(f"关节名: {self.joint_names}")
        self.get_logger().info("=" * 60)

    def controller_state_callback(self, msg):
        """处理控制器状态消息，提取reference字段的轨迹点信息"""
        self.get_logger().info(f"[收到控制器状态] 关节: {msg.joint_names}")

        # 创建命令消息
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = ""
        cmd_msg.name = self.joint_names

        # 初始化数组
        cmd_msg.position = [0.0] * 6
        cmd_msg.velocity = [0.0] * 6
        cmd_msg.effort = [0.0] * 6

        # 根据控制器状态的reference字段映射关节值
        for i, our_joint_name in enumerate(self.joint_names):
            try:
                controller_joint_idx = msg.joint_names.index(our_joint_name)
                if len(msg.reference.positions) > controller_joint_idx:
                    # 使用固定小数点格式，只保留小数点后4位，避免科学计数法
                    cmd_msg.position[i] = round(
                        msg.reference.positions[controller_joint_idx], 4
                    )
                if len(msg.reference.velocities) > controller_joint_idx:
                    # 使用固定小数点格式，只保留小数点后4位，避免科学计数法
                    cmd_msg.velocity[i] = round(
                        msg.reference.velocities[controller_joint_idx], 4
                    )
                # # 可选：使用加速度
                # if len(msg.reference.accelerations) > controller_joint_idx:
                #     # 使用固定小数点格式，只保留小数点后4位，避免科学计数法
                #     cmd_msg.effort[i] = msg.reference.accelerations[
                #         controller_joint_idx
                #     ]
            except ValueError:
                self.get_logger().warn(f"关节 {our_joint_name} 在控制器状态中未找到")

        # 检查是否所有值都为0，如果是则跳过发布
        if self._is_all_zero(cmd_msg) and not self.has_active_trajectory:
            self.get_logger().info("检测到全零消息，跳过发布")
            return

        # 如果position中有任意一个非零值，且velocity中有任意一个为0，则将为0的velocity替换为当前坐标的速度值
        if self._has_nonzero_position(cmd_msg.position) and self._has_zero_velocity(
            cmd_msg.velocity
        ):
            for i in range(6):
                # 重新获取当前关节的实际速度值，如果为0则使用最后大于0.01的速度值
                current_joint_idx = -1
                try:
                    current_joint_idx = msg.joint_names.index(self.joint_names[i])
                    actual_velocity = 0.0
                    if len(msg.reference.velocities) > current_joint_idx:
                        actual_velocity = round(
                            msg.reference.velocities[current_joint_idx], 4
                        )

                    if cmd_msg.velocity[i] == 0.0 and actual_velocity == 0.0:
                        # 如果从控制器获取的实际速度也为0，使用最后大于0.01的速度值
                        cmd_msg.velocity[i] = self.last_above_threshold_velocities[i]
                    elif abs(actual_velocity) > 0.5:
                        # 更新最后大于0.01的速度值，确保任何非零速度都被记录
                        self.last_above_threshold_velocities[i] = actual_velocity
                except ValueError:
                    # 如果在控制器状态中未找到该关节，使用最后大于0.01的速度值
                    cmd_msg.velocity[i] = self.last_above_threshold_velocities[i]

        # 检查是否有变化，如果有则发布到机械臂
        if (
            self._has_changed(cmd_msg.position, cmd_msg.velocity)
            or self.has_active_trajectory
        ):
            # 发布到机械臂
            for i in range(2):
                self.arm_cmd_pub.publish(cmd_msg)
            self.get_logger().info(
                f"已发送到机械臂: 位置={cmd_msg.position}, 速度={cmd_msg.velocity}"
            )

            # 更新最后发送的状态
            self.last_sent_positions = list(cmd_msg.position)
            self.last_sent_velocities = list(cmd_msg.velocity)

    def _has_nonzero_position(self, positions):
        """检查位置中是否有任意一个非零值"""
        return any(abs(pos) > 1e-9 for pos in positions)

    def _has_zero_velocity(self, velocities):
        """检查速度中是否有任意一个为0的值"""
        return 0.0 in velocities

    def _is_all_zero(self, joint_state_msg):
        """检查JointState消息是否所有位置、速度和努力值都为0"""
        # 检查位置是否全为0
        all_positions_zero = all(abs(pos) < 1e-9 for pos in joint_state_msg.position)

        # 检查速度是否全为0
        all_velocities_zero = all(abs(vel) < 1e-9 for vel in joint_state_msg.velocity)

        # 检查力矩是否全为0（如果effort数组有值）
        all_efforts_zero = True
        if joint_state_msg.effort:
            all_efforts_zero = all(abs(eff) < 1e-9 for eff in joint_state_msg.effort)

        # 如果位置、速度和力矩都为0，则返回True
        return all_positions_zero and all_velocities_zero and all_efforts_zero

    def _has_changed(self, positions, velocities, threshold=1e-6):
        """检查位置和速度是否发生了显著变化"""
        for i in range(6):
            if (
                abs(positions[i] - self.last_sent_positions[i]) > threshold
                or abs(velocities[i] - self.last_sent_velocities[i]) > threshold
            ):
                return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = A1XTrajectoryBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被用户终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
