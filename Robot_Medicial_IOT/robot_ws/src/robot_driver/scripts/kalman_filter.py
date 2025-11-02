import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from tf_transformations import quaternion_from_euler
import math
class KalmanFilter():
    def __init__(self):
        self.mu = np.zeros(5)
        
        self.sigma_sq = np.array([[0.1, 0, 0, 0, 0],
                                 [0, 0.1, 0, 0, 0],
                                 [0, 0, 0.1, 0, 0],
                                 [0, 0, 0, 0.1, 0],
                                 [0, 0, 0, 0, 0.1]])
        
        self.sigma_m_sq = np.array([[0.01, 0, 0, 0, 0],
                                   [0, 0.01, 0, 0, 0],
                                   [0, 0, 0.02, 0, 0],
                                   [0, 0, 0, 0.05, 0],
                                   [0, 0, 0, 0, 0.05]])

        self.sigma_z_sq = np.array([[0.05, 0, 0],
                                   [0, 0.1, 0],
                                   [0, 0, 0.02]])

        self.H = np.array([[0, 0, 0, 1, 0],
                          [0, 0, 0, 0, 1],
                          [0, 0, 0, 0, 1]])

        self.z_t = np.zeros(3)
        self.gyro_offset = None

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def calculate_odomcombine(self, v_odom, w_odom, w_imu, dt) -> tuple:
        self.z_t = np.array([v_odom, w_odom, w_imu])
        
        x_k, y_k, theta_k, v_k, w_k = self.mu
        
        self.mu = np.array([x_k + v_k * dt * math.cos(theta_k),
                           y_k + v_k * dt * math.sin(theta_k),
                           self.normalize_angle(theta_k + w_k * dt),
                           v_k,
                           w_k])
        F_k = np.array([[1, 0, -v_k*dt*math.sin(theta_k), dt*math.cos(theta_k), 0],
                       [0, 1, v_k*dt*math.cos(theta_k), dt*math.sin(theta_k), 0],
                       [0, 0, 1, 0, dt],
                       [0, 0, 0, 1, 0],
                       [0, 0, 0, 0, 1]])

        self.sigma_sq = F_k.dot(self.sigma_sq).dot(F_k.T) + self.sigma_m_sq
        
        measurement_residual = self.z_t - self.H.dot(self.mu)
        residual_covariance = self.H.dot(self.sigma_sq).dot(self.H.T) + self.sigma_z_sq
        K_t = self.sigma_sq.dot(self.H.T).dot(np.linalg.inv(residual_covariance))
        
        self.mu = self.mu + K_t.dot(measurement_residual)
        self.mu[2] = self.normalize_angle(self.mu[2])
        
        I_KH = np.eye(len(self.mu)) - K_t.dot(self.H)
        self.sigma_sq = I_KH.dot(self.sigma_sq).dot(I_KH.T) + K_t.dot(self.sigma_z_sq).dot(K_t.T)
        
        return (self.mu, self.sigma_sq)
