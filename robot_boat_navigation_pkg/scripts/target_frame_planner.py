#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

import math

# Laser scan is divided in 8 segments from 0 - 180 degrees
# Laser scan starts from -90 orientation

LASER_SECTOR_SIZE = 45
LASER_START_INDEX = 180
NUMBER_OF_SECTORS = 8
LASER_STOP_INDEX = LASER_SECTOR_SIZE * NUMBER_OF_SECTORS + LASER_START_INDEX

OBSTACLE_DISTANCE = 5.0
OBSTACLE_DISTANCE_X = 1.2

class LaserScanSector():
    __slots__ = ('sector_index',
                'contain_obstacle',
                'obstacle_distance_total',
                'obstacle_distance_x',
                'obstacle_distance_y')

    laser_sector_size = LASER_SECTOR_SIZE
    lser_start_index = LASER_START_INDEX
    laser_stop_index = LASER_STOP_INDEX

    def __init__(self, index):
        self.sector_index = index
        self.contain_obstacle = False
        self.obstacle_distance_total = 0.0
        self.obstacle_distance_x = 0.0
        self.obstacle_distance_y = 0.0


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


class TargetFramePlanner(Node):
    def __init__(self):
        super().__init__("target_frame_planner_node")
        self.sub_laser = self.create_subscription(LaserScan, 'robot_boat/laser_scan', self.sub_laser_clb, 2)

        # Subscriber to azimuth
        self.sub_pose_azimuth = self.create_subscription(PoseStamped, 'robot_boat/pose_p3d', self.azimuth_clb, 3)
        self.rotation_z = 90.0
        
        self.laser_scan_sector_array = [LaserScanSector(i) for i in range(NUMBER_OF_SECTORS)]

        self.br = tf2_ros.TransformBroadcaster(self)
        self.transform_stamped = TransformStamped()

        self.get_logger().info("target_frame_planner_node READY!")
    
    def azimuth_clb(self, msg):
        self.rotation_z = math.degrees(quaternion_to_euler(msg.pose.orientation)[2])


    def sub_laser_clb(self, msg):
        self.get_logger().info("NEW scan")
        # reset old data
        for i in self.laser_scan_sector_array:
            i.contain_obstacle = False 

        angle_increment = msg.angle_increment     
        for cnt, x in enumerate(msg.ranges[LASER_START_INDEX:LASER_STOP_INDEX:]):
            if (x < OBSTACLE_DISTANCE and x * math.cos(cnt * angle_increment)):
                self.laser_scan_sector_array[(int)(cnt / LaserScanSector.laser_sector_size)].contain_obstacle = True
                #print(f'Obstacle in {(int)(cnt / LaserScanSector.laser_sector_size)}',
                #        f' sector at {math.degrees(cnt * angle_increment):.2f} degrees',
                #        f" and distance_x {x * math.cos(cnt * angle_increment):.2f}")
            
        sector_occupancy_l = [int(i.contain_obstacle) for i in self.laser_scan_sector_array]
        print(f'Sectors: {sector_occupancy_l}')
        
        # Check all sectors and make decision
        if (sector_occupancy_l.count(0) == len(sector_occupancy_l)):
            # All secotors empty, go to global goal
            self.broadcast_new_tf()
            return
        if (sector_occupancy_l.count(1) == len(sector_occupancy_l)):
            # All secotors empty, go to global goal
            #self.broadcast_new_tf()
            self.get_logger().error("Error at finding target frame")
            return
        

        l = [0, 0, 0, 0, 0, 0, 0, 0]

        print(l)

        idx = 7
        cnt= 0 

        for i in range(len(l)):
            
            #if(idx == 7)
            
            
            #if (l[idx] == 1):
            #    #return idx
            #    pass
            
            idx = idx+cnt*(-1)**cnt
            if(idx > 7 or idx < 0):
                print(f'Not corr {idx}')
                cnt+=1
                idx = idx+cnt*(-1)**cnt
            print(idx)
            cnt+=1
        
        
     


        

        #self.broadcast_new_tf()
    
    def broadcast_new_tf(self, global_goal: bool = True, angle_deg: int = -1) -> None: 
        self.transform_stamped.child_frame_id = 'robot_boat/goal_frame'
        self.transform_stamped.header.frame_id = "robot_boat/odom"
        self.transform_stamped.header.frame_id = "base_link"
        self.transform_stamped.header.stamp = self.get_clock().now().to_msg()

        print(self.transform_stamped.header.stamp)

        if (angle_deg == -1):
            self.transform_stamped.transform.translation.x = 2.0
            self.transform_stamped.transform.translation.y = 0.0
        else:
            self.transform_stamped.transform.translation.x = OBSTACLE_DISTANCE * math.cos(angle_deg)
            self.transform_stamped.transform.translation.y = OBSTACLE_DISTANCE * math.sin(angle_deg)

        self.transform_stamped.transform.translation.z = 0.0
        self.transform_stamped.transform.rotation.x = 0.0
        self.transform_stamped.transform.rotation.y = 0.0
        self.transform_stamped.transform.rotation.z = 0.0
        self.transform_stamped.transform.rotation.w = 1.0

        self.br.sendTransform(self.transform_stamped)

def main():
    
    rclpy.init()

    target_frame_planner = TargetFramePlanner()

    while rclpy.ok():
        rclpy.spin(target_frame_planner)
    target_frame_planner.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()