#!/usr/bin/python3

import time
import numpy as np
from rotations import angle_normalize, rpy_jacobian_axis_angle, skew_symmetric, Quaternion
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped


# Define sensor variances
var_imu_f = 2.5e-2
var_imu_w = 0.5e-3
var_gnss = 10.
var_lidar = 2.5e-2

# Define constants
# g = np.array([0, 0, -9.8])  # Accounting for gravity
g = np.array([0, 0, 0])  # Ignore gravity.
l_jac = np.zeros([9, 6])
l_jac[3:, :] = np.eye(6)  # motion model noise jacobian
h_jac = np.zeros([3, 9])
h_jac[:, :3] = np.eye(3)  # measurement model jacobian


class EKF:
    """
    Implements an Error State Extended Kalman Filter (ES-EKF).

    Assumptions Made
    ----------------
        1. The ego-vehicle operates in a 2D plane perpendicular to the gravity vector.

    Parameters
    ----------
        ekf_p_pub: Instance of ROS Publisher on topic `/ekf_p` of PoseStamped messages.
                    The messages published contain the estimated position of the ego-vehicle.
        ekf_v_pub: Instance of ROS Publisher on topic `/ekf_v` of TwistStamped messages.
                    The messages published contain the estimated velocity of the ego-vehicle.
    """

    def __init__(self, ekf_p_pub, ekf_v_pub):
        # Define estimated states.
        self.p_est = np.zeros([3])
        self.v_est = np.zeros([3])
        self.q_est = Quaternion(euler=np.array([0, 0, 0])).to_numpy()
        self.p_cov_est = np.zeros([9, 9])
        # Define corrected states.
        self.p_km = np.zeros([3])
        self.v_km = np.zeros([3])
        self.q_km = Quaternion(euler=np.array([0, 0, 0])).to_numpy()
        self.p_cov_km = np.zeros([9, 9])

        self.bias_comp_counter = 0
        self.imu_f_bias = np.zeros([3])
        self.imu_w_bias = np.zeros([3])

        self.ekf_p_pub = ekf_p_pub
        self.pose_msg = PoseStamped()
        self.pose_msg.header.frame_id = "base_link"

        self.ekf_v_pub = ekf_v_pub
        self.twist_msg = TwistStamped()
        self.twist_msg.header.frame_id = "base_link"

        # Main Filter Loop
        self.F = np.identity(9)
        self.Q = np.identity(6)
        self.last_imu_time = None

    def pub_state(self, p, v, q):
        """
        Publish ego-state to cooresponding ROS topics.

        Parameters
        ----------
            p: Ego-state position.
            v: Ego-state velocity.
            q: Ego-state quaternion in NumPy representation.
        """

        quat = Quaternion(*q)
        self.pose_msg.pose.position.x = p[0]
        self.pose_msg.pose.position.y = p[1]
        # self.pose_msg.pose.position.z = p[2]
        self.pose_msg.pose.position.z = 0
        self.pose_msg.pose.orientation.w = quat.w
        self.pose_msg.pose.orientation.x = quat.x
        self.pose_msg.pose.orientation.y = quat.y
        self.pose_msg.pose.orientation.z = quat.z
        self.pose_msg.header.stamp = rospy.Time.now()

        self.twist_msg.twist.linear.x = v[0]
        self.twist_msg.twist.linear.y = v[1]
        self.twist_msg.twist.linear.z = v[2]
        self.twist_msg.header.stamp = rospy.Time.now()

        self.ekf_p_pub.publish(self.pose_msg)
        self.ekf_v_pub.publish(self.twist_msg)

    def imu_input(self, data):
        """
        Callback for /imu subscriber.
        Executes a predictive IMU update, updating the ego-state.

        Parameters
        ----------
            data: IMU data in ROS' sensor_msgs.IMU format.
        """

        accelerations = data.linear_acceleration
        rotations = data.angular_velocity
        # Ignore gravity (z = 0).
        imu_f = np.array([accelerations.x, accelerations.y, 0])
        # Only keep yaw measurement, assumes rover will operate in 2D plane.
        imu_w = np.array([0, 0, rotations.z])
        if self.bias_comp_counter < 200:  # While there are less than 200 raw IMU samples,
            # add IMU data to bias variables.
            self.imu_f_bias += imu_f
            self.imu_w_bias += imu_w
            self.bias_comp_counter += 1
        elif self.bias_comp_counter == 200:  # When 200 samples have been collected,
            # compute average of biases.
            self.imu_f_bias /= 200
            self.imu_w_bias /= 200
            self.bias_comp_counter += 1
        else:  # Else, IMU is set up and ready to go.
            delta_t = float(data.header.stamp.to_sec() - self.last_imu_time)
            self.imu_update(imu_f, imu_w, delta_t)

        self.last_imu_time = data.header.stamp.to_sec()

    def icp_input(self, data):
        """
        Callback for /icp subscriber.
        Executes a corrective update on the ego-state using more reliable positional updates.

        Parameters
        ----------
            data: ICP data in ROS' sensor_msgs.Vector3Stamped format.
        """

        icp_pos_data = data.vector
        icp_pos_data = np.array(
            [icp_pos_data.x, icp_pos_data.y, icp_pos_data.z])
        self.measurement_update(var_lidar, icp_pos_data)

    def measurement_update(self, sensor_var, y_k):
        """
        Perform corrective EKF update using data from GNSS or LiDAR.

        Parameters
        ----------
            sensor_var: Variance of sensor used to capture data represented in corrective data.
            y_k: Corrective data used to update ego-state.
        """
        # 3.1 Compute Kalman Gain
        I = np.identity(3)
        R = I * sensor_var
        K = self.p_cov_est.dot(h_jac.T).dot(np.linalg.inv(
            h_jac.dot(self.p_cov_est).dot(h_jac.T) + R))

        # 3.2 Compute error state
        error = K.dot(y_k - self.p_est)

        # 3.3 Correct predicted state
        p_del = error[:3]
        v_del = error[3:6]
        phi_del = error[6:]

        self.p_km = self.p_est + p_del
        self.v_km = self.v_est + v_del
        self.q_km = Quaternion(euler=phi_del).quat_mult_right(self.q_km)

        # 3.4 Compute corrected covariance
        self.p_cov_km = (np.identity(9) - K.dot(h_jac)).dot(self.p_cov_est)
        self.pub_state(self.p_km, self.v_km, self.q_km)

    def imu_update(self, imu_f, imu_w, delta_t):
        """
        Perform update of ego-state using incoming IMU acceleration and rotation data.

        Assumptions Made
        ----------------
            1. The effects of gravity have been negated.
                That is, the last value of imu_f (imu_f[2]) is set to a constant value of 0.

        Parameters
        ----------
            imu_f: 3D Vector containing IMU acceleration data.
            imu_w: 3D Vector containing IMU rotational velocity data.
            delta_t: Difference, in seconds, between current and previous IMU data.
        """

        imu_f -= self.imu_f_bias
        imu_w -= self.imu_w_bias

        # Rotation matrix cooresponding to current ego-orientation.
        C_ns = Quaternion(*self.q_km).normalize().to_mat()
        # Compute ego-state using available data inputs.
        self.p_est = self.p_km + (delta_t * self.v_km) + \
            (delta_t**2 / 2) * (C_ns @ imu_f + g)

        self.v_est = self.v_km + delta_t * (C_ns @ imu_f + g)
        self.q_est = Quaternion(
            axis_angle=imu_w * delta_t).quat_mult_right(self.q_km)

        # Linearize motion model and compute Jacobians.
        self.F = np.identity(9)
        self.F[:3, 3:6] = delta_t * np.identity(3)
        self.F[3:6, 6:] = -skew_symmetric(C_ns @ imu_f)
        self.Q = np.identity(6)
        self.Q[:, :3] *= var_imu_f * delta_t**2
        self.Q[:, 3:] *= var_imu_w * delta_t**2
        # Propagate uncertainty
        self.p_cov_est = self.F @ self.p_cov_km @ self.F.T + l_jac @ self.Q @ l_jac.T

        # Update previous ego-state before updating again.
        self.p_km = self.p_est.copy()
        self.v_km = self.v_est.copy()
        self.q_km = self.q_est.copy()
        self.p_cov_km = self.p_cov_est.copy()

        # Publish ego-state to cooresponding ROS topics.
        self.pub_state(self.p_km, self.v_km, self.q_km)
