#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import math
import numpy as np

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def quaternion_to_euler(q):
    q_x = q.x
    q_y = q.y
    q_z = q.z
    q_w = q.w

    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    # roll (x-axis rotation)
    sinr_cosp = 2 * (q_w * q_x + q_y * q_z)
    cosr_cosp = 1 - 2 * (q_x * q_x + q_y * q_y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (q_w * q_y - q_z * q_x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.PI / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (q_w * q_z + q_x * q_y)
    cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return [roll, pitch, yaw]


class CompassNode(Node):
    def __init__(self):
        super().__init__('compass_node')

        self.get_logger().info("Compass node started")

        # Subscribe to p3d_odom
        self.sub_p3d = self.create_subscription(Odometry, 'robot_boat/p3d', self.listener_clb, 3)

        # Publisher
        self.pub_pose_p3d = self.create_publisher(PoseStamped, 'robot_boat/pose_p3d', 3)
        self.pub_pose_azimuth = self.create_publisher(PoseStamped, 'robot_boat/pose_azimuth', 3)

    
    def listener_clb(self, msg):
         pose = PoseStamped()

         pose.header = msg.header
         pose.pose = msg.pose.pose

         self.pub_pose_p3d.publish(pose)

         rpy = quaternion_to_euler(msg.pose.pose.orientation)
         rpy[2] = 1.5708

         q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])

         pose.pose.orientation.x = q[0]
         pose.pose.orientation.y = q[1]
         pose.pose.orientation.z = q[2]
         pose.pose.orientation.w = q[3]
         self.pub_pose_azimuth.publish(pose)





def main():
    rclpy.init()

    compass_node = CompassNode()
    rclpy.spin(compass_node)
    compass_node.destroy_node()

    rclpy.shutdown()

if __name__=='__main__':
    main()