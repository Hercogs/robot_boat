
# ROS imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32

# KF filter imports
from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import JulierSigmaPoints
from filterpy.common import Q_discrete_white_noise

# General Python
import matplotlib.pyplot as plt
import numpy as np
import random


# F(X) for x,y,v
def fx5d(x, dt):
    xout = np.zeros_like(x)
    xout[0] = x[0] + np.sin(x[3] + x[4] * dt) * x[2] * dt
    xout[1] = x[1] + np.cos(x[3] + x[4] * dt) * x[2] * dt
    xout[2] = x[2]
    xout[3] = x[3] + x[4] * dt
    xout[4] = x[4]
    xout = np.round(xout, 5)
    return xout

# H(X) for x,y,v model
def hx5d(x):
    # returns x, y and theta
    return x[[0, 1, 3]]

# F(X) for x,vx,y,vy model
def fx6d(x, dt):
    xout = np.zeros_like(x)
    xout[0] = x[0] + x[1]*dt  # x
    xout[1] = x[1]  # vx
    xout[2] = x[2] + x[3]*dt  # y
    xout[3] = x[3]  # vy
    xout[4] = x[4] + x[5]*dt  # theta
    xout[5] = x[5]  # dtheta
    xout = np.round(xout, 5)
    return xout 

# H(x) for for x,vx,y,vy model
def hx6d(x):
    # returns x, y and theta
    return x[[0, 2, 4]]


class Ukf(Node):

    def __init__(self):
        super().__init__('ukf_gps_filter')

        # Node params
        self.MODEL = '5d'
        average_gps_msg_frequency = 0.925
        namespace = '/robot_boat'
        dim_z = 3

        # Node cross-variables
        self.__cmp_data = None
        self.__first = True
        self.zs = []
        self.fs = []
        self.gts = []

        # Initialize filter
        if self.MODEL == '5d':
            Q = np.array([
                [0.1, 0., 0., 0, 0.],
                [0., 0.1, 0., 0, 0.],
                [0.1, 0.1, 0.01, 0., 0.],
                [0., 0., 0., 0.1, 0.],
                [0., 0., 0., 0.1, 0.01],
            ])
            sigmas = JulierSigmaPoints(n=5, kappa=4)
            dim_x = 5
            fx = fx5d
            hx = hx5d

        elif self.MODEL == '6d':
            Q = Q_discrete_white_noise(dim=2, block_size=3)
            sigmas = JulierSigmaPoints(n=6, kappa=4)
            dim_x = 6
            fx = fx6d
            hx = hx6d
        else:
            raise Exception(
                "choose a valid filter m_model (expected: '5d' or '6d', received: {0}".format(self.MODEL))

        # This filter expects x: {x, y, v, th, dth} and z: {x, y, th}
        self.filter = UnscentedKalmanFilter(
            dim_x=dim_x,
            dim_z=dim_z,
            dt=1/average_gps_msg_frequency,
            hx=hx,
            fx=fx,
            points=sigmas
        )

        self.filter.P *= 10000
        self.filter.R *= np.array([
            [1., 0., 0.],
            [0., 1., 0.],
            [0., 0., .1]
        ])
        self.filter.Q = Q
        self.filter.predict()

        # Subscribe to GPS
        self.gps_sub = self.create_subscription(
            NavSatFix, namespace+'/gps_utm', self.gps_callback, 10)

        # Create UKF GPS publisher
        self.fgps_pub = self.create_publisher(
            NavSatFix, namespace + 'ukf_gps_utm', 3)

        # Subscribe to Compass
        self.cmp_sub = self.create_subscription(
            Int32, namespace+'/azimuth_north', self.cmp_callback, 10)

    def gps_callback(self, gps_msg: NavSatFix):
        # in UTM corrdinates:
        #   latitude = x
        #   longitude = y

        if self.__cmp_data is None:
            return

        xgt = gps_msg.latitude
        ygt = gps_msg.longitude

        xz = xgt + random.uniform(-1, 1)
        yz = ygt + random.uniform(-1, 1)

        # pack into measurement vector
        z = np.round(np.array([xz, yz, np.deg2rad(self.__cmp_data+180)]), 5)

        # on first measurement reset the UKF initial state to first measurement
        if self.__first:
            self.filter.x[2] = 0
            self.filter.x[0] = z[0]
            self.filter.x[1] = z[1]
            self.filter.x[3] = z[2]

        # UKF predict and update step
        self.filter.predict()
        self.filter.update(z)

        # Get GPS predicted by UKF
        if self.MODEL == '5d':
            xf = self.filter.x[0]
            yf = self.filter.x[1]
        elif self.MODEL == '6d':
            xf = self.filter.x[0]
            yf = self.filter.x[2]


        # plot settings
        LIMIT_TRACK_LIST_NUM = 300
        PLOT_AROUND_BOAT_POINT = False
        PLOT_GLOBAL_TRACK = True
        
        if PLOT_AROUND_BOAT_POINT or PLOT_GLOBAL_TRACK:
            # process GT data
            self.gts.append([xgt, ygt])
            if len(self.gts) > LIMIT_TRACK_LIST_NUM:
                self.gts.pop(0)
            data_gts = np.array(self.gts)

            # process GPS data
            self.zs.append([xz, yz])
            if len(self.gts) > LIMIT_TRACK_LIST_NUM:
                self.zs.pop(0)
            z_error = np.sqrt(np.mean((xgt-xz)**2 + (ygt-yz)**2))
            data_zs = np.array(self.zs)

            # process Filter predict
            self.fs.append([xf, yf])
            if len(self.fs) > LIMIT_TRACK_LIST_NUM:
                self.fs.pop(0)
            f_error = np.sqrt(np.mean((xgt-xf)**2 + (ygt-yf)**2))
            data_fs = np.array(self.fs)

            cmp_head = np.array(
                [[xgt, ygt], [xgt + 0.5*np.sin(z[2]), ygt+0.5*np.cos(z[2])]])

            plt.clf()
            plt.grid()


        if PLOT_AROUND_BOAT_POINT:
            # === PLOT AROUND BOAT POINT ===
            plt.xlim([xgt+2.5, xgt-2.5])
            plt.ylim([ygt+2.5, ygt-2.5])

            # -- FILTER --
            plt.plot(data_fs[:, 0], data_fs[:, 1], c='m')

            # last filter state measurement
            plt.scatter(xf, yf, marker='x', c='m', s=50)

            # -- GPS --
            # plt.plot(data_zs[:, 0], data_zs[:, 1], c='b') # track of last 5 GPS measurements
            # last GPS measurement
            plt.scatter(xz, yz, marker='x', c='b', s=50)
            plt.scatter(np.mean(data_zs[:, 0]), np.mean(
                data_zs[:, 1]), marker='o', c='b', s=50)  # mean of last 5 GPS measurements

            # -- GROUND TRUTH --
            plt.scatter(xgt, ygt, marker='x', c='g')  # ground truth marker
            # ground truth track last 5 points
            plt.plot(data_gts[:, 0], data_gts[:, 1])
            plt.plot(cmp_head[:, 0], cmp_head[:, 1], c='r')  # Compass heading

        if PLOT_GLOBAL_TRACK:
            #  === PLOT GLOBAL TRACK ===
            # -- FILTER --
            plt.plot(data_fs[:, 0], data_fs[:, 1], c='m')  # filter track
            # last filter state measurement
            plt.scatter(xf, yf, marker='x', c='m', s=50)

            # -- GROUND TRUTH ---
            plt.plot(data_gts[:, 0], data_gts[:, 1])  # true track
            plt.plot(cmp_head[:, 0], cmp_head[:, 1], c='r')  # Compass heading

            # -- GPS --
            plt.scatter(data_zs[:,0], data_zs[:,1], marker='x', c='b', s=50)

            plt.gca().set_aspect('equal')

        if PLOT_AROUND_BOAT_POINT or PLOT_GLOBAL_TRACK:
            plt.draw()
            plt.pause(0.0001)

            # first predict is really bad, so, for ease of plot, it gets removed
            if self.__first:
                self.__first = False
                self.zs.pop(0)
                self.gts.pop(0)
                self.fs.pop(0)

        # publish the new estimated gps
        utm_msg = NavSatFix()
        utm_msg.header = gps_msg.header
        utm_msg.header.stamp = self.get_clock().now().to_msg()
        utm_msg.latitude = xf
        utm_msg.longitude = yf
        self.fgps_pub.publish(utm_msg)

        # print('-- dash:')
        # print("z_error: {0}, f_error: {1}".format(z_error, f_error))

    def cmp_callback(self, cmp_msg: Int32):
        self.__cmp_data = cmp_msg.data


def main(args=None):
    rclpy.init()
    ukf_node = Ukf()
    rclpy.spin(ukf_node)

    ukf_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# /robot_boat/azimuth_north (need to add noise)
# /robot_boat/gps_utm (need to add noise)
