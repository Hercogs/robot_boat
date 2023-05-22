#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import subprocess
import os


# install dependencies for utm Node
res = subprocess.check_output(["pip3", "install", "utm"])
for line in res.splitlines():
    # process the output line by line
    print(line)




def generate_launch_description():

    robot_boat_gazebo_pkg = get_package_share_directory('robot_boat_gazebo_pkg')


    spawn_robot_ros2_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(robot_boat_gazebo_pkg, 'launch', 'spawn_robot_ros2.launch.xml')
        )
    )


    gps_utm_node = Node(
        package='robot_boat_gazebo_pkg',
        executable='gps_to_utm.py',
        output='screen',
        name='utm_node',
        emulate_tty=True
    )

    return LaunchDescription([
        spawn_robot_ros2_launch,
        gps_utm_node
    ])