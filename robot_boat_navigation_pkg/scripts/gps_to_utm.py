#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import utm
import random


class GPSSubscriber(Node):
    def __init__(self):
        super().__init__('gps_subscriber')

        namespace = 'robot_boat'

        # Create subscriber
        self.subscription = self.create_subscription(NavSatFix, 'robot_boat/gps', self.listener_clb, 3)

        # Create publisher
        self.publisher = self.create_publisher(NavSatFix, namespace + '/gps_utm', 3)

    

    # Callback function
    def listener_clb(self, msg):

        utm_msg = NavSatFix()
        utm_msg = msg

        u = utm.from_latlon(msg.latitude, msg.longitude)

        utm_msg.latitude = u[0] + random.uniform(-1, 1)
        utm_msg.longitude = u[1] + random.uniform(-1, 1)

        self.publisher.publish(utm_msg)

        




def main():
    rclpy.init()

    gps_subscriber = GPSSubscriber()
    rclpy.spin(gps_subscriber)

    gps_subscriber.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()

