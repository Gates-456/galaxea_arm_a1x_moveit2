#!/usr/bin/env python3
"""
A1X 机械臂 MoveIt2 执行桥接
功能：从控制器状态获取轨迹信息，转发至机械臂驱动接口。
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Empty, Float64
import numpy as np
import threading


class CubicSpline:
    """三次样条插值类，用于平滑轨迹点"""

    def __init__(self):
        self.x_sample = None  # 采样点x坐标（时间）
        self.y_sample = None  # 采样点y坐标（关节位置）
        self.M = None  # 二阶偏导数
        self.sample_count = 0
        self.bound1 = 0
        self.bound2 = 0
        self.vel = 0  # 插值点的速度
        self.acc = 0  # 插值点的加速度

    def load_data(self, x_data, y_data, count, bound1=0, bound2=0):
        """加载数据并计算样条插值
        Args:
            x_data: 时间点数组
            y_data: 位置点数组
            count: 数据点数量
            bound1: 左边界条件（一阶导数）
            bound2: 右边界条件（一阶导数）
        Returns:
            bool: 是否成功
        """
        if (x_data is None) or (y_data is None) or (count < 3):
            return False

        self.x_sample = np.array(x_data[:count])
        self.y_sample = np.array(y_data[:count])
        self.sample_count = count
        self.bound1 = bound1
        self.bound2 = bound2

        return self._spline()

    def _spline(self):
        """计算三次样条插值
        Returns:
            bool: 是否成功
        """
        # 追赶法解方程求二阶偏导数
        f1, f2 = self.bound1, self.bound2

        a = np.zeros(self.sample_count)  # 稀疏矩阵最下边一串数
        b = np.zeros(self.sample_count)  # 稀疏矩阵最中间一串数
        c = np.zeros(self.sample_count)  # 稀疏矩阵最上边一串数
        d = np.zeros(self.sample_count)
        f = np.zeros(self.sample_count)
        bt = np.zeros(self.sample_count)
        gm = np.zeros(self.sample_count)
        h = np.zeros(self.sample_count)

        b[:] = 2  # 中间一串数为2

        for i in range(self.sample_count - 1):
            if self.x_sample[i + 1] == self.x_sample[i]:
                h[i] = 0.005
            else:
                h[i] = self.x_sample[i + 1] - self.x_sample[i]  # 各段步长

        for i in range(1, self.sample_count - 1):
            a[i] = h[i - 1] / (h[i - 1] + h[i])
        a[self.sample_count - 1] = 1

        c[0] = 1
        for i in range(1, self.sample_count - 1):
            c[i] = h[i] / (h[i - 1] + h[i])

        for i in range(self.sample_count - 1):
            if self.x_sample[i + 1] == self.x_sample[i]:
                f[i] = (self.y_sample[i + 1] - self.y_sample[i]) / 0.005
            else:
                f[i] = (self.y_sample[i + 1] - self.y_sample[i]) / (
                    self.x_sample[i + 1] - self.x_sample[i]
                )

        for i in range(1, self.sample_count - 1):
            d[i] = 6 * (f[i] - f[i - 1]) / (h[i - 1] + h[i])

        # 追赶法求解方程（一阶导数边界条件）
        d[0] = 6 * (f[0] - f1) / h[0]
        d[self.sample_count - 1] = (
            6 * (f2 - f[self.sample_count - 2]) / h[self.sample_count - 2]
        )

        bt[0] = c[0] / b[0]
        for i in range(1, self.sample_count - 1):
            bt[i] = c[i] / (b[i] - a[i] * bt[i - 1])

        gm[0] = d[0] / b[0]
        for i in range(1, self.sample_count):
            gm[i] = (d[i] - a[i] * gm[i - 1]) / (b[i] - a[i] * bt[i - 1])

        self.M = np.zeros(self.sample_count)
        self.M[self.sample_count - 1] = gm[self.sample_count - 1]
        for i in range(self.sample_count - 2, -1, -1):
            self.M[i] = gm[i] - bt[i] * self.M[i + 1]

        return True

    def get_y_by_x(self, x_in):
        """根据x值获取插值后的y值、速度和加速度
        Args:
            x_in: 输入的时间值
        Returns:
            tuple: (y_out, vel, acc) 插值后的位置、速度和加速度
        """
        klo = 0
        khi = self.sample_count - 1

        # 二分法查找x所在区间段
        while khi - klo > 1:
            k = (khi + klo) >> 1
            if self.x_sample[k] > x_in:
                khi = k
            else:
                klo = k

        hh = self.x_sample[khi] - self.x_sample[klo]
        aa = (self.x_sample[khi] - x_in) / hh
        bb = (x_in - self.x_sample[klo]) / hh

        y_out = (
            aa * self.y_sample[klo]
            + bb * self.y_sample[khi]
            + ((aa * aa * aa - aa) * self.M[klo] + (bb * bb * bb - bb) * self.M[khi])
            * hh
            * hh
            / 6.0
        )

        # 计算加速度
        self.acc = (
            self.M[klo] * (self.x_sample[khi] - x_in)
            + self.M[khi] * (x_in - self.x_sample[klo])
        ) / hh

        # 计算速度
        self.vel = (
            self.M[khi]
            * (x_in - self.x_sample[klo])
            * (x_in - self.x_sample[klo])
            / (2 * hh)
            - self.M[klo]
            * (self.x_sample[khi] - x_in)
            * (self.x_sample[khi] - x_in)
            / (2 * hh)
            + (self.y_sample[khi] - self.y_sample[klo]) / hh
            - hh * (self.M[khi] - self.M[klo]) / 6
        )

        return y_out, self.vel, self.acc


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
            100,
        )

        # --- 发布到机械臂驱动接口 ---
        self.arm_cmd_pub = self.create_publisher(
            JointState, "/motion_target/target_joint_state_arm", 10
        )

        # 订阅停止命令
        self.move_stop_sub = self.create_subscription(
            Empty, "/rm_driver/move_stop_cmd", self.move_stop_callback, 10
        )

        # 订阅最小速度设置话题
        self.min_velocity_sub = self.create_subscription(
            Float64, "/min_velocity", self.min_velocity_callback, 10
        )

        # 存储上一次发送的关节状态，用于检测变化
        self.last_sent_positions = [0.0] * 6
        self.last_sent_velocities = [0.0] * 6

        # 存储每个关节的最后大于0.01的速度值
        self.last_above_threshold_velocities = [0.4] * 6

        # 添加标志来跟踪是否有活动轨迹
        self.has_active_trajectory = False

        # 插值相关参数
        self.interpolation_rate = 0.1  # 插值周期(s)，与C++代码中的rate一致
        self.min_interval = 0.0001  # 透传周期(s)
        self.wait_move_finish_time = 0.001  # 等待运动到位时间(s)
        self.min_velocity = 52.0  # 最小速度值，确保任意一个关节速度不为0
        self.count_keep_send = int(
            self.wait_move_finish_time / self.min_interval
        )  # 需要保持发送的次数
        self.count_final_joint = 0  # 当前已保持发送的次数

        # 存储插值后的轨迹点
        self.interpolated_trajectory = {
            "time": [],
            "positions": [[] for _ in range(6)],
            "velocities": [[] for _ in range(6)],
            "accelerations": [[] for _ in range(6)],
        }

        # 轨迹执行状态
        self.is_executing_trajectory = False
        self.current_trajectory_index = 0
        self.trajectory_lock = threading.Lock()

        # 创建定时器用于发送插值后的轨迹点
        self.trajectory_timer = self.create_timer(
            self.interpolation_rate, self.trajectory_timer_callback
        )

        self.get_logger().info("=" * 60)
        self.get_logger().info("A1X 轨迹桥接节点启动")
        self.get_logger().info(f"订阅: {self.controller_state_sub.topic_name}")
        self.get_logger().info(f"发布: {self.arm_cmd_pub.topic_name}")
        self.get_logger().info(f"关节名: {self.joint_names}")
        self.get_logger().info("=" * 60)

    def min_velocity_callback(self, msg):
        """处理最小速度设置话题的回调"""
        new_min_velocity = msg.data
        if new_min_velocity > 0:
            old_min_velocity = self.min_velocity
            self.min_velocity = new_min_velocity
            self.get_logger().info(
                f"最小速度从 {old_min_velocity} 更新为 {new_min_velocity}"
            )
        else:
            self.get_logger().warn(
                f"接收到无效的最小速度值: {new_min_velocity}，忽略此值"
            )

    def controller_state_callback(self, msg):
        """处理控制器状态消息，提取reference字段的轨迹点信息"""
        # self.get_logger().info(f"[收到控制器状态] 关节: {msg.joint_names}")

        # 提取轨迹点信息
        trajectory_points = []
        for i in range(len(msg.reference.positions)):
            point = {
                "positions": [0.0] * 6,
                "velocities": [0.0] * 6,
                "accelerations": [0.0] * 6,
                "time_from_start": 0.0,
            }

            for j, our_joint_name in enumerate(self.joint_names):
                try:
                    controller_joint_idx = msg.joint_names.index(our_joint_name)
                    if len(msg.reference.positions) > controller_joint_idx:
                        point["positions"][j] = round(
                            msg.reference.positions[controller_joint_idx], 4
                        )
                    if len(msg.reference.velocities) > controller_joint_idx:
                        point["velocities"][j] = round(
                            msg.reference.velocities[controller_joint_idx], 4
                        )
                        # 打印六个关节的最大速度值
                        # max_velocity = max(msg.reference.velocities)
                        # self.get_logger().info(f"六个关节的最大速度值: {max_velocity}")
                    if len(msg.reference.accelerations) > controller_joint_idx:
                        point["accelerations"][j] = round(
                            msg.reference.accelerations[controller_joint_idx], 4
                        )
                except ValueError:
                    self.get_logger().warn(
                        f"关节 {our_joint_name} 在控制器状态中未找到"
                    )

            # 假设每个点之间的时间间隔为0.1秒
            point["time_from_start"] = i * 0.1
            trajectory_points.append(point)

        # 检查是否有有效的轨迹点
        has_valid_trajectory = len(trajectory_points) > 0

        # 如果有足够的轨迹点，进行三次样条插值
        if len(trajectory_points) > 3:
            self._perform_cubic_spline_interpolation(trajectory_points)
        elif has_valid_trajectory:
            # 如果轨迹点较少，直接使用原始轨迹点
            with self.trajectory_lock:
                self.interpolated_trajectory["time"] = [
                    p["time_from_start"] for p in trajectory_points
                ]
                for j in range(6):
                    self.interpolated_trajectory["positions"][j] = [
                        p["positions"][j] for p in trajectory_points
                    ]
                    # 处理速度值，确保任意一个关节速度不为0，且最小速度为min_velocity
                    velocities = [p["velocities"][j] for p in trajectory_points]
                    for i in range(len(velocities)):
                        # 如果速度值小于min_velocity，则设置为min_velocity
                        if abs(velocities[i]) < self.min_velocity:
                            velocities[i] = (
                                self.min_velocity
                                if velocities[i] >= 0
                                else -self.min_velocity
                            )
                    self.interpolated_trajectory["velocities"][j] = velocities
                    self.interpolated_trajectory["accelerations"][j] = [
                        p["accelerations"][j] for p in trajectory_points
                    ]

            self.current_trajectory_index = 0
            self.is_executing_trajectory = True
            self.has_active_trajectory = True
        else:
            # 如果没有有效的轨迹点，重置执行状态
            with self.trajectory_lock:
                self.is_executing_trajectory = False
                self.has_active_trajectory = False

    def _perform_cubic_spline_interpolation(self, trajectory_points):
        """使用三次样条插值对轨迹进行平滑处理"""
        # 提取时间点和各关节的位置点
        times = [p["time_from_start"] for p in trajectory_points]
        max_time = times[-1]

        # 为每个关节进行三次样条插值
        for joint_idx in range(6):
            positions = [p["positions"][joint_idx] for p in trajectory_points]

            # 创建三次样条插值对象
            spline = CubicSpline()
            spline.load_data(times, positions, len(times), 0, 0)

            # 生成插值时间点
            x_out = -self.interpolation_rate
            interp_times = []
            interp_positions = []
            interp_velocities = []
            interp_accelerations = []

            while x_out < max_time:
                x_out += self.interpolation_rate
                y_out, vel, acc = spline.get_y_by_x(x_out)
                interp_times.append(x_out)
                interp_positions.append(round(y_out, 4))
                # 处理速度值，确保任意一个关节速度不为0，且最小速度为min_velocity
                vel_rounded = round(vel, 4)
                # 如果速度值小于min_velocity，则设置为min_velocity
                if abs(vel_rounded) < self.min_velocity:
                    vel_rounded = (
                        self.min_velocity if vel_rounded >= 0 else -self.min_velocity
                    )
                interp_velocities.append(vel_rounded)
                interp_accelerations.append(round(acc, 4))

            # 存储插值结果
            with self.trajectory_lock:
                if joint_idx == 0:  # 只需设置一次时间
                    self.interpolated_trajectory["time"] = interp_times

                self.interpolated_trajectory["positions"][joint_idx] = interp_positions
                self.interpolated_trajectory["velocities"][
                    joint_idx
                ] = interp_velocities
                self.interpolated_trajectory["accelerations"][
                    joint_idx
                ] = interp_accelerations

        # 重置轨迹执行状态
        self.current_trajectory_index = 0
        self.is_executing_trajectory = True
        self.has_active_trajectory = True
        self.count_final_joint = 0

    def trajectory_timer_callback(self):
        """定时器回调函数，用于发送插值后的轨迹点"""
        with self.trajectory_lock:
            # 只有在有活动轨迹时才发送消息
            if not self.is_executing_trajectory or not self.has_active_trajectory:
                return

            # 检查是否还有轨迹点需要发送
            if self.current_trajectory_index < len(
                self.interpolated_trajectory["time"]
            ):
                # 获取当前轨迹点
                idx = self.current_trajectory_index
                current_positions = [
                    self.interpolated_trajectory["positions"][j][idx] for j in range(6)
                ]

                # 检查位置是否发生显著变化
                position_changed = False
                for i in range(6):
                    if abs(current_positions[i] - self.last_sent_positions[i]) > 0.001:
                        position_changed = True
                        break

                # 只有当位置发生显著变化时才发送消息
                if position_changed:
                    # 创建命令消息
                    cmd_msg = JointState()
                    cmd_msg.header.stamp = self.get_clock().now().to_msg()
                    cmd_msg.header.frame_id = ""
                    cmd_msg.name = self.joint_names
                    cmd_msg.position = current_positions

                    # 处理速度值，确保任意一个关节速度不为0，且最小速度为min_velocity
                    velocities = [
                        self.interpolated_trajectory["velocities"][j][idx]
                        for j in range(6)
                    ]
                    for i in range(len(velocities)):
                        # 如果速度值小于min_velocity，则设置为min_velocity
                        if abs(velocities[i]) < self.min_velocity:
                            velocities[i] = (
                                self.min_velocity
                                if velocities[i] >= 0
                                else -self.min_velocity
                            )
                    cmd_msg.velocity = velocities

                    cmd_msg.effort = [
                        self.interpolated_trajectory["accelerations"][j][idx]
                        for j in range(6)
                    ]

                    # 发布到机械臂
                    self.arm_cmd_pub.publish(cmd_msg)

                    # 更新最后发送的位置和速度
                    self.last_sent_positions = list(current_positions)
                    self.last_sent_velocities = list(velocities)

                # 移动到下一个轨迹点
                self.current_trajectory_index += 1
            else:
                # 所有轨迹点已发送，继续发送最后一个点一段时间
                if self.count_final_joint < self.count_keep_send:
                    # 创建命令消息
                    cmd_msg = JointState()
                    cmd_msg.header.stamp = self.get_clock().now().to_msg()
                    cmd_msg.header.frame_id = ""
                    cmd_msg.name = self.joint_names

                    # 获取最后一个轨迹点
                    last_idx = len(self.interpolated_trajectory["time"]) - 1
                    cmd_msg.position = [
                        self.interpolated_trajectory["positions"][j][last_idx]
                        for j in range(6)
                    ]

                    # 处理速度值，确保任意一个关节速度不为0，且最小速度为min_velocity
                    velocities = [
                        self.interpolated_trajectory["velocities"][j][last_idx]
                        for j in range(6)
                    ]
                    for i in range(len(velocities)):
                        # 如果速度值小于min_velocity，则设置为min_velocity
                        if abs(velocities[i]) < self.min_velocity:
                            velocities[i] = (
                                self.min_velocity
                                if velocities[i] >= 0
                                else -self.min_velocity
                            )
                    cmd_msg.velocity = velocities

                    cmd_msg.effort = [
                        self.interpolated_trajectory["accelerations"][j][last_idx]
                        for j in range(6)
                    ]

                    for i in range(2):
                        # 发布到机械臂
                        self.arm_cmd_pub.publish(cmd_msg)

                    # 增加计数器
                    self.count_final_joint += 1
                else:
                    # 重置状态
                    self.count_final_joint = 0
                    self.current_trajectory_index = 0
                    self.is_executing_trajectory = False
                    self.has_active_trajectory = False

    def move_stop_callback(self, msg):
        """处理停止命令
        Args:
            msg: 空消息
        """
        with self.trajectory_lock:
            self.is_executing_trajectory = False
            self.has_active_trajectory = False
            self.current_trajectory_index = 0
            self.count_final_joint = 0

        self.get_logger().info("收到停止命令，停止轨迹执行")

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
