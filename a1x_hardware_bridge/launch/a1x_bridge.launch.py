import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 定义参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 定义桥接节点
    bridge_node = Node(
        package="a1x_hardware_bridge",
        executable="a1x_bridge",
        name="a1x_trajectory_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                # 如果以后需要修改话题名称，可以在这里直接改
                "command_topic": "/a1x_group_controller/joint_trajectory",
                "robot_feedback_topic": "/hdas/feedback_arm",
                "robot_command_topic": "/motion_target/target_joint_state_arm",
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        bridge_node,
    ])