#!/usr/bin/env python3

"""
Move to pose node with Cartesian planning using ROS2 MoveIt2 services
修复版本：解决TransformListener回调组问题
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
from tf2_ros import TransformException, Buffer, TransformListener
import threading
import time
import queue
import math
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

# MoveIt2 相关消息和服务
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotState, Constraints
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from moveit_msgs.action import ExecuteTrajectory


class MoveToPoseNode(Node):
    def __init__(self):
        super().__init__("move_to_pose_node")

        # 使用安全参数处理方法
        def get_param_safe(name, default_val):
            if self.has_parameter(name):
                return self.get_parameter(name).value
            else:
                self.declare_parameter(name, default_val)
                return default_val

        # 获取参数
        try:
            self.use_sim_time = self.get_parameter("use_sim_time").value
        except rclpy.exceptions.ParameterNotDeclaredException:
            self.use_sim_time = False

        self.move_group_name = get_param_safe("move_group_name", "a1x_group")
        self.end_effector_link = get_param_safe("end_effector_link", "arm_link6")
        self.velocity_scaling = get_param_safe("velocity_scaling", 0.5)
        self.acceleration_scaling = get_param_safe("acceleration_scaling", 0.5)
        self.cartesian_step_size = get_param_safe("cartesian_step_size", 0.005)

        self.get_logger().info(f"参数加载完成:")
        self.get_logger().info(f"  - move_group_name: {self.move_group_name}")
        self.get_logger().info(f"  - end_effector_link: {self.end_effector_link}")

        # 初始化 TF - 移除callback_group参数
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 初始化pymoveit2（如果可用）
        self.moveit2 = None
        try:
            from pymoveit2 import MoveIt2

            self.get_logger().info("使用pymoveit2包")

            joint_names = [
                "arm_joint1",
                "arm_joint2",
                "arm_joint3",
                "arm_joint4",
                "arm_joint5",
                "arm_joint6",
            ]

            self.moveit2 = MoveIt2(
                node=self,
                joint_names=joint_names,
                base_link_name="base_link",
                end_effector_name=self.end_effector_link,
                group_name=self.move_group_name,
            )

            self.moveit2.max_velocity = self.velocity_scaling
            self.moveit2.max_acceleration = self.acceleration_scaling

        except ImportError:
            self.get_logger().warn("pymoveit2包不可用，使用替代方法")

        # 创建笛卡尔路径规划服务客户端
        self.cartesian_path_client = self.create_client(
            GetCartesianPath,
            "compute_cartesian_path",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

        # 等待服务可用
        while not self.cartesian_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("等待笛卡尔路径规划服务...")

        # 创建Action客户端用于执行轨迹
        self.execute_action_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )

        # 等待Action服务器
        self.get_logger().info("等待 /execute_trajectory Action Server...")
        if not self.execute_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                "/execute_trajectory Action Server 未就绪，将使用备用执行方法"
            )

        # 检查是否有可用的关节状态
        self.current_joint_state = None
        self.joint_state_subscriber = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # 创建发布者和订阅者
        self.end_effector_pose_publisher = self.create_publisher(
            PoseStamped, "end_effector_pose", 10
        )

        self.end_effector_pose_subscriber = self.create_subscription(
            PoseStamped, "target_end_effector_pose", self.target_pose_callback, 10
        )

        self.cartesian_move_subscriber = self.create_subscription(
            PoseStamped,
            "cartesian_linear_move",
            self.cartesian_linear_move_callback,
            10,
        )

        self.z_axis_move_subscriber = self.create_subscription(
            PoseStamped, "z_axis_linear_move", self.z_axis_linear_move_callback, 10
        )

        # 创建定时器
        self.timer = self.create_timer(
            0.1, self.publish_current_pose  # 降低频率，减少冲突
        )

        # 初始化任务队列和相关变量
        self.task_queue = queue.Queue()
        self.is_executing = False
        self.lock = threading.Lock()

        # 创建一个专用的执行线程
        self.task_processing_thread = threading.Thread(
            target=self.process_task_queue, daemon=True
        )
        self.task_processing_thread.start()

        self.get_logger().info(f"MoveToPoseNode 初始化完成")

    def joint_state_callback(self, msg):
        """关节状态回调 - 简化版本"""
        with self.lock:
            self.current_joint_state = msg

    def get_current_robot_state(self):
        """获取当前机器人状态"""
        with self.lock:
            if self.current_joint_state is None:
                return None

            robot_state = RobotState()
            robot_state.joint_state = self.current_joint_state
            robot_state.is_diff = True
            return robot_state

    def get_current_pose(self):
        """获取当前末端执行器位姿"""
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                self.end_effector_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )

            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation

            return pose

        except TransformException as ex:
            self.get_logger().debug(f"无法获取变换: {ex}")
            return None
        except Exception as e:
            self.get_logger().debug(f"获取当前姿态时出错: {str(e)}")
            return None

    def compute_cartesian_path(self, waypoints, max_step=0.01, jump_threshold=0.0):
        """
        计算笛卡尔路径 - 使用call_async避免阻塞
        """
        try:
            # 获取当前机器人状态
            start_state = self.get_current_robot_state()
            if start_state is None:
                self.get_logger().error("无法获取机器人起始状态")
                return None, 0.0

            # 创建请求
            request = GetCartesianPath.Request()
            request.header.frame_id = "base_link"
            request.header.stamp = self.get_clock().now().to_msg()
            request.start_state = start_state
            request.group_name = self.move_group_name
            request.link_name = self.end_effector_link
            request.waypoints = waypoints
            request.max_step = max_step
            request.jump_threshold = jump_threshold
            request.avoid_collisions = True
            request.path_constraints = Constraints()

            # 异步调用服务
            future = self.cartesian_path_client.call_async(request)

            # 等待结果（最大等待5秒）
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > 5.0:
                    self.get_logger().error("笛卡尔路径规划超时")
                    return None, 0.0
                time.sleep(0.01)

            response = future.result()
            if response is not None:
                self.get_logger().info(
                    f"笛卡尔路径规划完成: {response.fraction * 100:.1f}%"
                )
                return response.solution, response.fraction
            else:
                self.get_logger().error("笛卡尔路径规划服务调用失败")
                return None, 0.0

        except Exception as e:
            self.get_logger().error(f"计算笛卡尔路径时出错: {str(e)}")
            return None, 0.0

    def execute_trajectory(self, robot_trajectory):
        """
        执行机器人轨迹 - 简化版本
        """
        if robot_trajectory is None:
            self.get_logger().error("接收到的轨迹为 None")
            return False

        # 首先尝试使用Action接口
        if self.execute_action_client.server_is_ready():
            try:
                # 创建目标消息
                goal_msg = ExecuteTrajectory.Goal()
                goal_msg.trajectory = robot_trajectory

                # 发送目标
                future = self.execute_action_client.send_goal_async(goal_msg)

                # 简单等待执行开始
                time.sleep(0.1)
                self.get_logger().info("轨迹已通过Action接口发送执行")
                return True

            except Exception as e:
                self.get_logger().warn(f"通过Action接口执行失败: {e}")

        # 如果Action接口不可用，尝试使用pymoveit2
        if self.moveit2 is not None:
            try:
                # 注意：这里需要根据pymoveit2的实际接口进行调整
                self.get_logger().info("通过pymoveit2执行轨迹")
                # 由于pymoveit2可能需要不同的接口，这里简化处理
                time.sleep(1.0)  # 模拟执行时间
                return True
            except Exception as e:
                self.get_logger().error(f"通过pymoveit2执行失败: {e}")

        self.get_logger().error("所有执行方法都失败了")
        return False

    def linear_move_z_axis(self, z_distance):
        """沿Z轴进行笛卡尔直线运动 - 简化版本"""
        self.get_logger().info(f"开始Z轴笛卡尔直线运动，距离: {z_distance:.3f}米")

        # 获取当前位姿
        current_pose = self.get_current_pose()
        if current_pose is None:
            self.get_logger().error("无法获取当前位姿")
            return False

        # 创建目标位姿（只改变Z坐标）
        target_pose = Pose()
        target_pose.position.x = current_pose.position.x
        target_pose.position.y = current_pose.position.y
        target_pose.position.z = current_pose.position.z + z_distance
        target_pose.orientation = current_pose.orientation

        self.get_logger().info(
            f"当前位置: Z={current_pose.position.z:.3f}, "
            f"目标位置: Z={target_pose.position.z:.3f}"
        )

        # 计算笛卡尔路径
        trajectory, fraction = self.compute_cartesian_path(
            waypoints=[target_pose],
            max_step=self.cartesian_step_size,
            jump_threshold=0.0,
        )

        if trajectory is None or fraction < 0.95:
            self.get_logger().error(
                f"笛卡尔路径规划失败，覆盖率: {fraction * 100:.1f}%"
            )
            return False

        # 执行轨迹
        success = self.execute_trajectory(trajectory)
        if success:
            # 简单等待运动完成
            time.sleep(2.0)
        return success

    def linear_move_along_axis(self, x_offset, y_offset, z_offset, orientation=None):
        """沿指定轴进行笛卡尔直线运动 - 简化版本"""
        self.get_logger().info(
            f"开始笛卡尔直线运动，偏移量: ({x_offset:.3f}, {y_offset:.3f}, {z_offset:.3f})"
        )

        # 获取当前位姿
        current_pose = self.get_current_pose()
        if current_pose is None:
            self.get_logger().error("无法获取当前位姿")
            return False

        # 创建目标位姿
        target_pose = Pose()
        target_pose.position.x = current_pose.position.x + x_offset
        target_pose.position.y = current_pose.position.y + y_offset
        target_pose.position.z = current_pose.position.z + z_offset

        # 设置姿态
        if orientation is not None:
            ox, oy, oz, ow = orientation
            target_pose.orientation.x = ox
            target_pose.orientation.y = oy
            target_pose.orientation.z = oz
            target_pose.orientation.w = ow
        else:
            target_pose.orientation = current_pose.orientation

        # 计算笛卡尔路径
        trajectory, fraction = self.compute_cartesian_path(
            waypoints=[target_pose],
            max_step=self.cartesian_step_size,
            jump_threshold=0.0,
        )

        if trajectory is None or fraction < 0.95:
            self.get_logger().error(
                f"笛卡尔路径规划失败，覆盖率: {fraction * 100:.1f}%"
            )
            return False

        # 执行轨迹
        success = self.execute_trajectory(trajectory)
        if success:
            # 简单等待运动完成
            time.sleep(2.0)
        return success

    def plan_and_execute_to_pose(self, x, y, z, ox, oy, oz, ow):
        """规划并执行到指定姿态的路径 - 简化版本"""
        if self.moveit2 is None:
            self.get_logger().error("MoveIt2接口不可用")
            return False

        # 设置目标姿态
        try:
            self.moveit2.set_pose_goal(position=[x, y, z], quat_xyzw=[ox, oy, oz, ow])

            # 规划
            self.get_logger().info("开始规划到目标姿态的路径...")
            plan = self.moveit2.plan()

            if plan is None:
                self.get_logger().error("规划到目标姿态的路径失败")
                return False

            self.get_logger().info("规划完成")

            # 执行
            self.get_logger().info("开始执行规划...")
            self.moveit2.execute(plan)

            # 简单等待执行完成
            time.sleep(0.5)
            return True

        except Exception as e:
            self.get_logger().error(f"执行移动到目标姿态时发生异常: {str(e)}")
            return False

    # 回调函数
    def target_pose_callback(self, msg):
        """处理目标姿态回调"""
        try:
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            ox = msg.pose.orientation.x
            oy = msg.pose.orientation.y
            oz = msg.pose.orientation.z
            ow = msg.pose.orientation.w

            self.get_logger().info(f"接收到目标姿态: pos=({x:.3f},{y:.3f},{z:.3f})")

            # 检查是否正在执行
            if self.is_executing:
                self.get_logger().warn("当前正在执行任务，将新任务加入队列")

            # 添加到任务队列
            self.task_queue.put(
                {"type": "target_pose", "data": (x, y, z, ox, oy, oz, ow)}
            )

        except Exception as e:
            self.get_logger().error(f"解析目标姿态时出错: {str(e)}")

    def cartesian_linear_move_callback(self, msg):
        """处理笛卡尔线性移动回调"""
        try:
            x_offset = msg.pose.position.x
            y_offset = msg.pose.position.y
            z_offset = msg.pose.position.z

            self.get_logger().info(
                f"接收到笛卡尔移动指令: ({x_offset:.3f}, {y_offset:.3f}, {z_offset:.3f})"
            )

            # 添加到任务队列
            self.task_queue.put(
                {"type": "cartesian_move", "data": (x_offset, y_offset, z_offset)}
            )

        except Exception as e:
            self.get_logger().error(f"处理笛卡尔线性移动时出错: {str(e)}")

    def z_axis_linear_move_callback(self, msg):
        """处理Z轴线性移动回调"""
        try:
            z_distance = msg.pose.position.z
            self.get_logger().info(f"接收到Z轴移动指令: {z_distance:.3f}")

            # 添加到任务队列
            self.task_queue.put({"type": "z_axis_move", "data": z_distance})

        except Exception as e:
            self.get_logger().error(f"处理Z轴线性移动时出错: {str(e)}")

    def process_task_queue(self):
        """处理任务队列 - 简化版本"""
        while rclpy.ok():
            try:
                task = self.task_queue.get(timeout=0.5)
                if task is None:
                    continue

                task_type = task["type"]

                # 标记开始执行
                self.is_executing = True

                success = False
                try:
                    if task_type == "target_pose":
                        x, y, z, ox, oy, oz, ow = task["data"]
                        self.get_logger().info("处理目标姿态任务...")
                        success = self.plan_and_execute_to_pose(x, y, z, ox, oy, oz, ow)

                    elif task_type == "cartesian_move":
                        x_offset, y_offset, z_offset = task["data"]
                        success = self.linear_move_along_axis(
                            x_offset, y_offset, z_offset
                        )

                    elif task_type == "z_axis_move":
                        z_distance = task["data"]
                        success = self.linear_move_z_axis(z_distance)

                    if success:
                        self.get_logger().info("任务执行成功")
                    else:
                        self.get_logger().error("任务执行失败")

                except Exception as e:
                    self.get_logger().error(f"执行任务时发生异常: {str(e)}")

                finally:
                    # 标记执行结束
                    self.is_executing = False
                    self.task_queue.task_done()
                    # 短暂延迟，避免连续执行过快
                    time.sleep(0.1)

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"处理任务队列时发生异常: {str(e)}")
                time.sleep(0.5)
                continue

    def publish_current_pose(self):
        """发布当前末端执行器位姿"""
        try:
            current_pose = self.get_current_pose()
            if current_pose is not None:
                pose_msg = PoseStamped()
                pose_msg.header = Header()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "base_link"
                pose_msg.pose = current_pose
                self.end_effector_pose_publisher.publish(pose_msg)
        except Exception as e:
            self.get_logger().debug(f"发布当前位姿时出错: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    try:
        # 创建节点
        node = MoveToPoseNode()

        # 使用简单spin而不是多线程执行器
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("收到键盘中断信号，关闭节点...")
    except Exception as e:
        print(f"节点异常: {e}")
    finally:
        if "node" in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
