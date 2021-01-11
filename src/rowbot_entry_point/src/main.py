#!/usr/bin/python3
import time
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan, Imu, NavSatFix
from geometry_msgs.msg import TwistStamped

import numpy as np
import numba as nb

from ekf_module import EKF


def _lidar_input(data):
    np_range_data = np.array(data.ranges)
    np_angle_data = np.arange(
        data.angle_min, data.angle_max, data.angle_increment)
    return np_angle_data, np_range_data


def lidar_input(data):
    global i
    x, y = _lidar_input(data)
    # rospy.loginfo(F"\nData: \n{data}")
    # rospy.loginfo(F"\nNumpy: \n{np_range_data}")
    # rospy.loginfo("%s, %s", data.header.stamp, type(data))
    rospy.loginfo(i)
    i += 1


def on_input(data):
    rospy.loginfo("%s, \t%s", data.header.stamp.to_sec(), data)
    # rospy.loginfo(i)
    # i += 1


def listener():
    # anonymous=True flag allows for multiple `listener` nodes.
    myEKF = EKF()
    rospy.init_node('ekf_listener', anonymous=True)
    # rospy.Subscriber("scan", LaserScan, on_input)
    # rospy.Subscriber("imu", Imu, myEKF.imu_input)
    rospy.Subscriber("imu", TwistStamped, myEKF.imu_input)
    # rospy.Subscriber("gps", NavSatFix, on_input)
    rospy.spin()  # Keeps Python from exiting until this node is stopped.


if __name__ == '__main__':
    listener()
