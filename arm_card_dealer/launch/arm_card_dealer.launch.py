from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_card_dealer',
            executable='arm_card_dealer_node',
            name='arm_card_dealer_node',
            output='screen',
            parameters=[
                {'move_group_name': 'a1x_group'},
                {'end_effector_link': 'arm_link6'},
                {'use_sim_time': False}
            ]
        )
    ])