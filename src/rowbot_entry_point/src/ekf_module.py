#!/usr/bin/python3
import time
# from numba import jitclass
# from numba import float32
import numpy as np
from rotations import angle_normalize, rpy_jacobian_axis_angle, skew_symmetric, Quaternion


# Define sensor variances
var_imu_f = 1e-6
var_imu_w = 1e-6
var_gnss = 0.01
var_lidar = 1e-3
# Define constants
g = np.array([0, 0, -9.8])  # gravity
l_jac = np.zeros([9, 6])
l_jac[3:, :] = np.eye(6)  # motion model noise jacobian
h_jac = np.zeros([3, 9])
h_jac[:, :3] = np.eye(3)  # measurement model jacobian


class EKF:
    def __init__(self):
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

        # Main Filter Loop
        self.F = np.identity(9)
        self.Q = np.identity(6)
        self.last_imu_time = None

    def imu_input(self, data):
        accelerations = data.twist.linear
        rotations = data.twist.angular
        if self.last_imu_time:
            delta_t = float(data.header.stamp.to_sec() - self.last_imu_time)
            # print(F"Delta_t: {delta_t}")
            imu_f = np.array(
                [accelerations.x, accelerations.y, accelerations.z])
            imu_w = np.array([rotations.x, rotations.y, rotations.z])
            self.imu_update(imu_f, imu_w, delta_t)

        self.last_imu_time = data.header.stamp.to_sec()

    def icp_input(self, data):
        pass

    def measurement_update(self, sens_var, y_k):
        # 3.1 Compute Kalman Gain
        R = sens_var * np.identity(3)
        K = self.p_cov_est @ h_jac.T @ np.linalg.inv(
            h_jac @ self.p_cov_est @ h_jac.T + R)

        # 3.2 Compute error state
        error_state = K @ (y_k - self.p_est)

        # 3.3 Correct predicted state
        self.p_km = self.p_est + error_state[:3]
        self.v_km = self.v_est + error_state[3:6]
        self.q_km = Quaternion(
            axis_angle=error_state[6:]).quat_mult_left(self.q_km)

        # 3.4 Compute corrected covariance
        self.p_cov_km = (np.identity(9) - K @ h_jac) @ self.p_cov_km

    def imu_update(self, imu_f, imu_w, delta_t):
        print(F"My best positional guess: {self.p_km}")
        C_ns = Quaternion(*self.q_km).normalize().to_mat()
        self.p_est = self.p_km + (delta_t * self.v_km) + \
            (delta_t**2 / 2) * (C_ns @ imu_f + g)
        self.v_est = self.v_km + delta_t * (C_ns @ imu_f + g)
        self.q_est = Quaternion(
            axis_angle=imu_w * delta_t).quat_mult_right(self.q_km)
        # Linearize motion model and compute Jacobians.
        self.F[:3, 3:6] = delta_t * np.identity(3)
        self.F[3:6, 6:] = -skew_symmetric(C_ns @ imu_f) * delta_t
        self.Q[:3, :3] = var_imu_f * delta_t**2 * np.identity(3)
        self.Q[3:, 3:] = var_imu_w * delta_t**2 * np.identity(3)
        # Propagate uncertainty
        self.p_cov_est = self.F @ self.p_cov_km @ self.F.T + l_jac @ self.Q @ l_jac.T

        self.p_km = self.p_est
        self.v_km = self.v_est
        self.q_km = self.q_est
        self.p_cov_km = self.p_cov_est
