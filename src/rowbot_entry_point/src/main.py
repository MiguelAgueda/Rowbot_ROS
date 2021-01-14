#!/usr/bin/python3
import time
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan, Imu, NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3Stamped

import numpy as np
import numba as nb

from ekf_module import EKF


def listener():
    ekf_p_pub = rospy.Publisher("/ekf_p", PoseStamped, queue_size=10)
    ekf_v_pub = rospy.Publisher("/ekf_v", TwistStamped, queue_size=10)
    # anonymous=True flag allows for multiple `listener` nodes.
    myEKF = EKF(ekf_p_pub, ekf_v_pub)
    rospy.init_node('ekf_listener', anonymous=True)
    # rospy.Subscriber("scan", LaserScan, on_input)
    rospy.Subscriber("imu", Imu, myEKF.imu_input)
    rospy.Subscriber("icp", Vector3Stamped, myEKF.icp_input)
    # rospy.Subscriber("imu", TwistStamped, myEKF.imu_input)
    # rospy.Subscriber("gps", NavSatFix, on_input)
    rospy.spin()  # Keeps Python from exiting until this node is stopped.


if __name__ == '__main__':
    listener()
