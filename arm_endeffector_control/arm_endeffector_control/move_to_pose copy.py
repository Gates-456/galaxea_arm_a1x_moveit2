#!/usr/bin/env python3

"""
Move to pose node with Cartesian planning using ROS2 MoveIt2 services
终极稳定版本：修复wait set index too big错误
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
from rclpy.callback_groups import ReentrantCallbackGroup
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

        # 使用ReentrantCallbackGroup，但限制并发
        self.callback_group = ReentrantCallbackGroup()

        # 初始化 TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 关节状态管理
        self.joint_state_lock = threading.Lock()
        self.current_joint_state = None
        self.last_joint_state_time = time.time()
        self.joint_state_received = False

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

            self.get_logger().info("pymoveit2初始化完成")

        except ImportError:
            self.get_logger().warn("pymoveit2包不可用，使用替代方法")
        except Exception as e:
            self.get_logger().error(f"pymoveit2初始化失败: {str(e)}")
            self.moveit2 = None

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
        self.get_logger().info("等待笛卡尔路径规划服务...")
        while not self.cartesian_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("等待笛卡尔路径规划服务...")

        # 创建Action客户端用于执行轨迹
        self.execute_action_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )

        # 检查是否有可用的关节状态 - 使用回调组
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
            callback_group=self.callback_group,
        )

        # 创建发布者和订阅者 - 使用回调组
        self.end_effector_pose_publisher = self.create_publisher(
            PoseStamped, "end_effector_pose", 10
        )

        self.end_effector_pose_subscriber = self.create_subscription(
            PoseStamped,
            "target_end_effector_pose",
            self.target_pose_callback,
            10,
            callback_group=self.callback_group,
        )

        self.cartesian_move_subscriber = self.create_subscription(
            PoseStamped,
            "cartesian_linear_move",
            self.cartesian_linear_move_callback,
            10,
            callback_group=self.callback_group,
        )

        self.z_axis_move_subscriber = self.create_subscription(
            PoseStamped,
            "z_axis_linear_move",
            self.z_axis_linear_move_callback,
            10,
            callback_group=self.callback_group,
        )

        # 创建定时器 - 使用回调组
        self.timer = self.create_timer(
            0.1,  # 10Hz更新频率
            self.publish_current_pose,
            callback_group=self.callback_group,
        )

        # 状态监控定时器 - 使用回调组
        self.monitor_timer = self.create_timer(
            5.0,  # 每5秒监控一次
            self.monitor_status,
            callback_group=self.callback_group,
        )

        # 初始化任务队列
        self.task_queue = queue.Queue()
        self.task_processing = False
        self.task_lock = threading.Lock()

        # 启动任务处理线程
        self.task_thread = threading.Thread(
            target=self.process_tasks_safely, daemon=True, name="task_processor"
        )
        self.task_thread.start()

        # 性能统计
        self.stats = {
            "plans_success": 0,
            "plans_failed": 0,
            "executions_success": 0,
            "executions_failed": 0,
            "total_tasks": 0,
            "start_time": time.time(),
        }

        # 错误计数器
        self.error_count = 0
        self.last_error_time = 0

        self.get_logger().info("MoveToPoseNode 初始化完成 - 终极稳定版本")

    def joint_state_callback(self, msg):
        """关节状态回调"""
        try:
            with self.joint_state_lock:
                self.current_joint_state = msg
                self.last_joint_state_time = time.time()
                self.joint_state_received = True
        except Exception as e:
            self.log_error(f"关节状态回调异常: {str(e)}")

    def get_current_robot_state(self):
        """获取当前机器人状态"""
        try:
            with self.joint_state_lock:
                if self.current_joint_state is None:
                    return None

                # 检查关节状态是否过时（超过1.0秒）
                if time.time() - self.last_joint_state_time > 1.0:
                    return None

                robot_state = RobotState()
                robot_state.joint_state = self.current_joint_state
                robot_state.is_diff = True
                return robot_state
        except Exception as e:
            self.log_error(f"获取机器人状态异常: {str(e)}")
            return None

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

        except TransformException:
            return None
        except Exception as e:
            self.log_error(f"获取当前姿态异常: {str(e)}")
            return None

    def compute_cartesian_path(self, waypoints, max_step=0.01, jump_threshold=0.0):
        """
        计算笛卡尔路径
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

            # 同步调用服务
            future = self.cartesian_path_client.call_async(request)

            # 等待结果（最大3秒）
            start_time = time.time()
            while rclpy.ok() and not future.done():
                if time.time() - start_time > 3.0:
                    self.get_logger().error("笛卡尔路径规划超时")
                    return None, 0.0
                time.sleep(0.01)

            response = future.result()
            if response is not None:
                return response.solution, response.fraction
            else:
                self.get_logger().error("笛卡尔路径规划服务调用失败")
                return None, 0.0

        except Exception as e:
            self.log_error(f"计算笛卡尔路径异常: {str(e)}")
            return None, 0.0

    def execute_trajectory(self, robot_trajectory):
        """
        执行机器人轨迹
        """
        if robot_trajectory is None:
            self.get_logger().error("接收到的轨迹为 None")
            return False

        try:
            # 创建目标消息
            goal_msg = ExecuteTrajectory.Goal()
            goal_msg.trajectory = robot_trajectory

            # 发送目标
            future = self.execute_action_client.send_goal_async(goal_msg)

            # 等待发送完成（最大1秒）
            start_time = time.time()
            while rclpy.ok() and not future.done():
                if time.time() - start_time > 1.0:
                    self.get_logger().warn("发送轨迹目标超时")
                    return False
                time.sleep(0.01)

            if future.done():
                try:
                    goal_handle = future.result()
                    if goal_handle.accepted:
                        return True
                    else:
                        self.get_logger().warn("轨迹目标被拒绝")
                        return False
                except Exception as e:
                    self.log_error(f"获取轨迹执行结果异常: {e}")
                    return False

        except Exception as e:
            self.log_error(f"通过Action接口执行异常: {e}")

        return False

    def linear_move_z_axis(self, z_distance):
        """沿Z轴进行笛卡尔直线运动"""
        try:
            self.get_logger().info(f"Z轴移动: {z_distance:.3f}米")

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
            return self.execute_trajectory(trajectory)

        except Exception as e:
            self.log_error(f"Z轴移动异常: {str(e)}")
            return False

    def linear_move_along_axis(self, x_offset, y_offset, z_offset, orientation=None):
        """沿指定轴进行笛卡尔直线运动"""
        try:
            self.get_logger().info(
                f"笛卡尔移动: ({x_offset:.3f}, {y_offset:.3f}, {z_offset:.3f})"
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
            return self.execute_trajectory(trajectory)

        except Exception as e:
            self.log_error(f"笛卡尔移动异常: {str(e)}")
            return False

    def plan_and_execute_to_pose(self, x, y, z, ox, oy, oz, ow):
        """规划并执行到指定姿态的路径"""
        if self.moveit2 is None:
            self.get_logger().error("MoveIt2接口不可用")
            return False

        try:
            # 设置目标姿态
            self.moveit2.set_pose_goal(position=[x, y, z], quat_xyzw=[ox, oy, oz, ow])

            # 规划
            plan = self.moveit2.plan()

            if plan is None:
                self.get_logger().error("规划失败")
                self.stats["plans_failed"] += 1
                return False

            self.stats["plans_success"] += 1

            # 执行规划
            self.moveit2.execute(plan)

            self.stats["executions_success"] += 1
            return True

        except Exception as e:
            self.log_error(f"规划执行异常: {str(e)}")
            self.stats["executions_failed"] += 1
            return False

    def process_tasks_safely(self):
        """安全处理任务"""
        while rclpy.ok():
            try:
                # 获取任务
                task = self.task_queue.get(timeout=0.1)
                if task is None:
                    continue

                self.stats["total_tasks"] += 1

                with self.task_lock:
                    self.task_processing = True

                try:
                    task_type = task["type"]
                    task_data = task["data"]

                    success = False

                    if task_type == "target_pose":
                        x, y, z, ox, oy, oz, ow = task_data
                        success = self.plan_and_execute_to_pose(x, y, z, ox, oy, oz, ow)

                    elif task_type == "cartesian_move":
                        x_offset, y_offset, z_offset = task_data
                        success = self.linear_move_along_axis(
                            x_offset, y_offset, z_offset
                        )

                    elif task_type == "z_axis_move":
                        z_distance = task_data
                        success = self.linear_move_z_axis(z_distance)

                    if not success:
                        self.get_logger().error(f"任务执行失败: {task_type}")

                except Exception as e:
                    self.log_error(f"任务处理异常: {str(e)}")

                finally:
                    with self.task_lock:
                        self.task_processing = False
                    self.task_queue.task_done()

            except queue.Empty:
                continue
            except Exception as e:
                self.log_error(f"任务队列异常: {str(e)}")
                time.sleep(0.1)

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

            # 静默模式：每10个任务记录一次
            if self.stats["total_tasks"] % 10 == 0:
                self.get_logger().info(f"目标姿态: ({x:.3f}, {y:.3f}, {z:.3f})")

            # 添加到任务队列
            self.task_queue.put(
                {"type": "target_pose", "data": (x, y, z, ox, oy, oz, ow)}
            )

        except Exception as e:
            self.log_error(f"解析目标姿态异常: {str(e)}")

    def cartesian_linear_move_callback(self, msg):
        """处理笛卡尔线性移动回调"""
        try:
            x_offset = msg.pose.position.x
            y_offset = msg.pose.position.y
            z_offset = msg.pose.position.z

            # 添加到任务队列
            self.task_queue.put(
                {"type": "cartesian_move", "data": (x_offset, y_offset, z_offset)}
            )

        except Exception as e:
            self.log_error(f"处理笛卡尔线性移动异常: {str(e)}")

    def z_axis_linear_move_callback(self, msg):
        """处理Z轴线性移动回调"""
        try:
            z_distance = msg.pose.position.z

            # 添加到任务队列
            self.task_queue.put({"type": "z_axis_move", "data": z_distance})

        except Exception as e:
            self.log_error(f"处理Z轴线性移动异常: {str(e)}")

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
            self.log_error(f"发布当前位姿异常: {str(e)}", level="debug")

    def monitor_status(self):
        """监控系统状态"""
        try:
            uptime = time.time() - self.stats["start_time"]

            # 计算成功率
            total_plans = self.stats["plans_success"] + self.stats["plans_failed"]
            plan_success_rate = (
                (self.stats["plans_success"] / total_plans * 100)
                if total_plans > 0
                else 100
            )

            total_executions = (
                self.stats["executions_success"] + self.stats["executions_failed"]
            )
            exec_success_rate = (
                (self.stats["executions_success"] / total_executions * 100)
                if total_executions > 0
                else 100
            )

            status_msg = (
                f"运行时间: {uptime:.0f}s | "
                f"规划: {plan_success_rate:.1f}% | "
                f"执行: {exec_success_rate:.1f}% | "
                f"任务: {self.stats['total_tasks']} | "
                f"队列: {self.task_queue.qsize()} | "
                f"错误: {self.error_count}"
            )

            self.get_logger().info(status_msg)

        except Exception as e:
            self.log_error(f"监控状态异常: {str(e)}", level="debug")

    def log_error(self, message, level="error"):
        """记录错误，避免错误风暴"""
        current_time = time.time()

        # 限制错误记录频率
        if current_time - self.last_error_time > 1.0:  # 每秒最多记录一次
            self.error_count += 1
            self.last_error_time = current_time

            if level == "error":
                self.get_logger().error(message)
            elif level == "warn":
                self.get_logger().warn(message)
            elif level == "debug":
                self.get_logger().debug(message)


def main(args=None):
    rclpy.init(args=args)

    try:
        # 使用多线程执行器，但限制线程数
        executor = MultiThreadedExecutor(num_threads=2)
        node = MoveToPoseNode()

        executor.add_node(node)

        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info("收到键盘中断信号，关闭节点...")
        except Exception as e:
            node.get_logger().error(f"执行器异常: {str(e)}")

    except Exception as e:
        print(f"节点初始化异常: {e}")
    finally:
        if "node" in locals():
            node.destroy_node()
        if "executor" in locals():
            executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
