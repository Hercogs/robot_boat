import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

import xacro

# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    urdf_file = 'robot_boat.urdf'
    xacro_file = "robot_boat.xacro"
    package_description = "robot_boat_description_pkg"

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", xacro_file)

    # convert XACRO file into URDF
    doc = xacro.parse(open(robot_desc_path))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # Robot State Publisher

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        #parameters=[{'use_sim_time': True, 'robot_description': doc.toxml()}],
        output="screen"
    )

    # RVIZ Configuration
    #rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'urdf_vis.rviz')


    #rviz_node = Node(
    #        package='rviz2',
    #        executable='rviz2',
    #        output='screen',
    #        name='rviz_node',
    #        parameters=[{'use_sim_time': True}],
    #        arguments=['-d', rviz_config_dir])

    # create and return launch description object
    return LaunchDescription(
        [            
            robot_state_publisher_node
            #rviz_node
        ]
    )