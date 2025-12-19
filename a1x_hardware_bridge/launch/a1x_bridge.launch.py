import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 定义桥接节点
    bridge_node = Node(
        package="a1x_hardware_bridge",
        executable="a1x_bridge",
        name="a1x_bridge_node",
        output="screen",
        parameters=[
            {
                # 如果以后需要修改话题名称，可以在这里直接改
                "command_topic": "/joint_states",
                "robot_feedback_topic": "/hdas/feedback_arm",
                "robot_command_topic": "/motion_target/target_joint_state_arm",
            }
        ],
    )

    return LaunchDescription([bridge_node])
