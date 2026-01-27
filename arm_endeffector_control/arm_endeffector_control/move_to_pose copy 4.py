#!/usr/bin/env python3

"""
Move to pose node with Cartesian planning using ROS2 MoveIt2 services
修复版本：解决joint_state属性设置问题
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

        # 初始化 TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # TF状态跟踪
        self.last_tf_update = time.time()
        self.tf_update_interval = 0.0

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
            
            # 首先收集关节名称（从参数或默认）
            joint_names = [
                "arm_joint1",
                "arm_joint2",
                "arm_joint3",
                "arm_joint4",
                "arm_joint5",
                "arm_joint6",
            ]

            # 创建MoveIt2实例 - 使用更简单的初始化
            self.moveit2 = MoveIt2(
                node=self,
                joint_names=joint_names,
                base_link_name="base_link",
                end_effector_name=self.end_effector_link,
                group_name=self.move_group_name,
            )
            
            self.moveit2.max_velocity = self.velocity_scaling
            self.moveit2.max_acceleration = self.acceleration_scaling
            
            self.get_logger().info("pymoveit2初始化完成，等待关节状态...")
            
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
            )
        )

        # 等待服务可用
        self.get_logger().info("等待笛卡尔路径规划服务...")
        while not self.cartesian_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("等待笛卡尔路径规划服务...")

        # 创建Action客户端用于执行轨迹
        self.execute_action_client = ActionClient(
            self,
            ExecuteTrajectory,
            "/execute_trajectory"
        )

        # 检查是否有可用的关节状态
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
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
            10
        )

        self.z_axis_move_subscriber = self.create_subscription(
            PoseStamped, "z_axis_linear_move", self.z_axis_linear_move_callback, 10
        )

        # 创建定时器
        self.timer = self.create_timer(
            0.05,  # 20Hz更新频率
            self.publish_current_pose
        )

        # 状态监控定时器
        self.monitor_timer = self.create_timer(
            2.0,  # 2Hz状态监控
            self.monitor_status
        )

        # 初始化任务队列
        self.task_queue = queue.Queue()
        self.task_lock = threading.Lock()
        self.current_task = None
        self.task_thread = None
        
        # 启动任务处理线程
        self.start_task_processor()

        # 性能统计
        self.stats = {
            "plans_success": 0,
            "plans_failed": 0,
            "executions_success": 0,
            "executions_failed": 0,
            "total_tasks": 0,
            "start_time": time.time()
        }

        self.get_logger().info("MoveToPoseNode 初始化完成 - 修复版本")

    def joint_state_callback(self, msg):
        """关节状态回调"""
        with self.joint_state_lock:
            self.current_joint_state = msg
            self.last_joint_state_time = time.time()
            self.joint_state_received = True

    def get_current_robot_state(self):
        """获取当前机器人状态"""
        with self.joint_state_lock:
            if self.current_joint_state is None:
                return None
                
            # 检查关节状态是否过时（超过0.5秒）
            if time.time() - self.last_joint_state_time > 0.5:
                return None
                
            robot_state = RobotState()
            robot_state.joint_state = self.current_joint_state
            robot_state.is_diff = True
            return robot_state

    def get_current_pose(self):
        """获取当前末端执行器位姿"""
        try:
            current_time = time.time()
            # 计算TF更新间隔
            if hasattr(self, 'last_tf_update'):
                self.tf_update_interval = current_time - self.last_tf_update
            self.last_tf_update = current_time
            
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                self.end_effector_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )

            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation

            return pose

        except TransformException as ex:
            # 定期记录TF错误
            if time.time() - getattr(self, '_last_tf_error_log', 0) > 5.0:
                self.get_logger().debug(f"TF变换异常: {ex}")
                self._last_tf_error_log = time.time()
            return None
        except Exception as e:
            self.get_logger().debug(f"获取当前姿态时出错: {str(e)}")
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
            while not future.done():
                if time.time() - start_time > 3.0:
                    self.get_logger().error("笛卡尔路径规划超时")
                    return None, 0.0
                time.sleep(0.01)
            
            response = future.result()
            if response is not None:
                self.get_logger().info(f"笛卡尔路径规划完成: {response.fraction * 100:.1f}%")
                return response.solution, response.fraction
            else:
                self.get_logger().error("笛卡尔路径规划服务调用失败")
                return None, 0.0

        except Exception as e:
            self.get_logger().error(f"计算笛卡尔路径时出错: {str(e)}")
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
            
            # 发送目标并等待接受
            future = self.execute_action_client.send_goal_async(goal_msg)
            
            # 等待发送完成
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > 2.0:
                    self.get_logger().warn("发送轨迹目标超时")
                    return False
                time.sleep(0.01)
            
            if future.done():
                goal_handle = future.result()
                if goal_handle.accepted:
                    self.get_logger().info("轨迹已发送执行")
                    return True
                else:
                    self.get_logger().warn("轨迹目标被拒绝")
                    return False
                    
        except Exception as e:
            self.get_logger().error(f"通过Action接口执行失败: {str(e)}")
            
        return False

    def linear_move_z_axis(self, z_distance):
        """沿Z轴进行笛卡尔直线运动"""
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

        self.get_logger().debug(
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
        return success

    def linear_move_along_axis(self, x_offset, y_offset, z_offset, orientation=None):
        """沿指定轴进行笛卡尔直线运动"""
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
        return success

    def plan_and_execute_to_pose(self, x, y, z, ox, oy, oz, ow):
        """规划并执行到指定姿态的路径"""
        if self.moveit2 is None:
            self.get_logger().error("MoveIt2接口不可用")
            return False

        try:
            # 设置目标姿态
            self.moveit2.set_pose_goal(position=[x, y, z], quat_xyzw=[ox, oy, oz, ow])
            
            # 规划 - 添加超时和错误处理
            self.get_logger().info("开始规划到目标姿态的路径...")
            start_time = time.time()
            
            plan = None
            try:
                plan = self.moveit2.plan()
            except Exception as e:
                self.get_logger().error(f"规划过程中发生异常: {str(e)}")
                return False
            
            planning_time = time.time() - start_time
            
            if plan is None:
                self.get_logger().error("规划到目标姿态的路径失败")
                self.stats["plans_failed"] += 1
                return False
                
            self.get_logger().info(f"规划完成，耗时: {planning_time:.3f}s")
            self.stats["plans_success"] += 1
            
            # 执行规划
            self.get_logger().info("开始执行规划...")
            try:
                self.moveit2.execute(plan)
                
                # 等待执行完成
                time.sleep(0.1)
                
                self.stats["executions_success"] += 1
                return True
                
            except Exception as e:
                self.get_logger().error(f"执行规划时出错: {str(e)}")
                self.stats["executions_failed"] += 1
                return False
            
        except Exception as e:
            self.get_logger().error(f"执行移动到目标姿态时发生异常: {str(e)}")
            return False

    def start_task_processor(self):
        """启动任务处理器"""
        def task_processor():
            while rclpy.ok():
                try:
                    # 获取任务
                    task = self.task_queue.get(timeout=0.1)
                    if task is None:
                        continue
                    
                    self.stats["total_tasks"] += 1
                    
                    with self.task_lock:
                        self.current_task = task
                        
                    task_type = task["type"]
                    task_data = task["data"]
                    
                    success = False
                    
                    if task_type == "target_pose":
                        x, y, z, ox, oy, oz, ow = task_data
                        self.get_logger().info("处理目标姿态任务...")
                        success = self.plan_and_execute_to_pose(x, y, z, ox, oy, oz, ow)
                        
                    elif task_type == "cartesian_move":
                        x_offset, y_offset, z_offset = task_data
                        success = self.linear_move_along_axis(x_offset, y_offset, z_offset)
                        
                    elif task_type == "z_axis_move":
                        z_distance = task_data
                        success = self.linear_move_z_axis(z_distance)
                    
                    if success:
                        self.get_logger().info("任务执行成功")
                    else:
                        self.get_logger().error("任务执行失败")
                        
                    with self.task_lock:
                        self.current_task = None
                        
                    self.task_queue.task_done()
                    
                    # 短暂延迟，避免连续执行过快
                    time.sleep(0.05)
                    
                except queue.Empty:
                    continue
                except Exception as e:
                    self.get_logger().error(f"任务处理器异常: {str(e)}")
                    with self.task_lock:
                        self.current_task = None
                    self.task_queue.task_done()
        
        # 启动线程
        self.task_thread = threading.Thread(
            target=task_processor,
            daemon=True,
            name="task_processor"
        )
        self.task_thread.start()

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

            self.get_logger().info(
                f"接收到目标姿态: pos=({x:.3f},{y:.3f},{z:.3f})"
            )

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
            # 仅在严重错误时记录
            if not hasattr(self, '_last_pose_error') or time.time() - self._last_pose_error > 10.0:
                self.get_logger().debug(f"发布当前位姿时出错: {str(e)}")
                self._last_pose_error = time.time()

    def monitor_status(self):
        """监控系统状态"""
        uptime = time.time() - self.stats["start_time"]
        
        # 计算TF更新率
        tf_rate = 1.0 / self.tf_update_interval if self.tf_update_interval > 0 else 0
        
        status_msg = (
            f"状态报告:\n"
            f"  运行时间: {uptime:.1f}s\n"
            f"  TF更新率: {tf_rate:.1f}Hz\n"
            f"  关节状态: {'已接收' if self.joint_state_received else '未接收'}\n"
            f"  规划成功: {self.stats['plans_success']}\n"
            f"  规划失败: {self.stats['plans_failed']}\n"
            f"  执行成功: {self.stats['executions_success']}\n"
            f"  执行失败: {self.stats['executions_failed']}\n"
            f"  总任务数: {self.stats['total_tasks']}\n"
            f"  队列大小: {self.task_queue.qsize()}"
        )
        
        self.get_logger().info(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        # 创建节点
        node = MoveToPoseNode()
        
        # 使用简单的spin
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("收到键盘中断信号，关闭节点...")
    except Exception as e:
        node.get_logger().error(f"节点异常: {str(e)}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()