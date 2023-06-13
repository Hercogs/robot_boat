#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ukf_node = Node(
        package='filter_pkg',
        namespace='robot_boat',
        executable='ukf_node',
        output='screen',
        name='ukf_node',
        emulate_tty=True
    )

    return LaunchDescription([
        ukf_node
    ])