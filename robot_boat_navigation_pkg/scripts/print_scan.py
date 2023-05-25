#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SubLaser(Node):
    def __init__(self):
        super().__init__("laser_node")
        subscriber = self.create_subscription(LaserScan, 'robot_boat/laser_scan', self.clb, 2)


    def clb(self, msg):
        print(msg.ranges)

def main():
    
    rclpy.init()

    xx = SubLaser()

    while rclpy.ok():
        rclpy.spin(xx)
    
    xx.destroy_node()
    
    rclpy.shutdown()
    # MAIN

if __name__=='__main__':
    main()