import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 定义参数
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # 定义关节状态重映射节点
    joint_state_remap_node = Node(
        package="a1x_hardware_bridge",
        executable="joint_state_remap",
        name="joint_state_remap",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use sim time if true",
            ),
            joint_state_remap_node,
        ]
    )
