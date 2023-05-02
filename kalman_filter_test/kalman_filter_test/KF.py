"""
    Author: Aditya Ramesh
    Implements Kalman filter for ellipse detection and pose detection
"""

import numpy as np

class EKF_pose_estimation(object):
    """Extended Kalman filter for circle pose detection"""
    def __init__(self, dt, std_acc, x_std_meas, y_std_meas, z_std_meas, nx_std_meas, ny_std_meas, nz_std_meas):
        """
            dt: sampling time (time for 1 cycle)
            std_acc: process noise magnitude
            x_std_meas: standard deviation of the measurement in x-direction
            y_std_meas: standard deviation of the measurement in y-direction
            z_std_meas: standard deviation of the measurement in z-direction
            nx_std_meas: standard deviation of the measurement in normal vector x-direction
            ny_std_meas: standard deviation of the measurement in normal vector y-direction
            nz_std_meas: standard deviation of the measurement in normal vector z-direction
        """

        self.dt = dt
        
        # Initial state x
        # state: [x, v_x, y, v_y, z, v_z, nx, v_nx, ny, n_ny, nz, n_nz]
        self.x = np.matrix([[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]])

        # Inital covariance matrix P
        self.P = np.eye(12)

        # Measurement Noise matrix R
        self.R = np.matrix([[x_std_meas**2, 0, 0, 0, 0, 0],
                            [0, y_std_meas**2, 0, 0, 0, 0],
                            [0, 0, z_std_meas**2, 0, 0, 0],
                            [0, 0, 0, nx_std_meas**2, 0, 0],
                            [0, 0, 0, 0, ny_std_meas**2, 0],
                            [0, 0, 0, 0, 0, nz_std_meas**2]])
        
        # Process Noise Covariance Q
        self.Q = np.matrix([[(dt**4)/4, (dt**3)/2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [(dt**3)/2, (dt**2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, (dt**4)/4, (dt**3)/2, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, (dt**3)/2, (dt**2), 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, (dt**4)/4, (dt**3)/2, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, (dt**3)/2, (dt**2), 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, (dt**4)/4, (dt**3)/2, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, (dt**3)/2, (dt**2), 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, (dt**4)/4, (dt**3)/2, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, (dt**3)/2, (dt**2), 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, (dt**4)/4, (dt**3)/2],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, (dt**3)/2, (dt**2)]] * (std_acc**2))
        
        # Jacobian matrix of measurement mapping function 
        # normally in filter update but is constant so definined in init
        self.H = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0]])
    
    def filter(self, u, z):
        """
            u: input vector
            z: measurement vector
        """
        u_x, u_y, u_z, u_nx, u_ny, u_nz, v_x, v_y, w_z = u

        [x_prev, v_x_prev, y_prev, v_y_prev, z_prev, v_z_prev, 
        nx_prev, v_nx_prev, ny_prev, n_ny_prev, nz_prev, n_nz_prev] = np.asarray(self.x).flatten().tolist()
        dt = self.dt
        theta = w_z * dt

        self.x = np.matrix([[u_x*(dt**2)/2 + v_x*dt + x_prev*np.cos(theta) - y_prev*np.sin(theta)],
                            [u_x*dt + v_x +  v_x_prev*np.cos(theta) - v_y_prev*np.sin(theta)],
                            [u_y*(dt**2)/2 + v_y*dt + x_prev*np.sin(theta) + y_prev*np.cos(theta)],
                            [u_y*dt + v_y +  v_x_prev*np.sin(theta) + v_y_prev*np.cos(theta)],
                            [u_z*(dt**2)/2],
                            [u_z*dt],
                            [u_nx*(dt**2)/2 + v_x*dt + nx_prev*np.cos(theta) - ny_prev*np.sin(theta)],
                            [u_nx*dt + v_x +  v_nx_prev*np.cos(theta) - n_ny_prev*np.sin(theta)],
                            [u_ny*(dt**2)/2 + v_y*dt + nx_prev*np.sin(theta) + ny_prev*np.cos(theta)],
                            [u_ny*dt + v_y +  v_nx_prev*np.sin(theta) + n_ny_prev*np.cos(theta)],
                            [u_nz*(dt**2)/2],
                            [u_nz*dt]])


        self.G = np.matrix([[np.cos(theta), 0, -np.sin(theta), 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, np.cos(theta), 0, -np.sin(theta), 0, 0, 0, 0, 0, 0, 0, 0],
                            [np.sin(theta), 0, np.cos(theta), 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, np.sin(theta), 0, np.cos(theta), 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, np.cos(theta), 0, -np.sin(theta), 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, np.cos(theta), 0, -np.sin(theta), 0, 0],
                            [0, 0, 0, 0, 0, 0, np.sin(theta), 0, np.cos(theta), 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, np.sin(theta), 0, np.cos(theta), 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])


        self.P = self.G @ self.P @ self.G.T + self.Q
        K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
        self.x = self.x + K @ (z - self.H @ self.x)
        self.P = (np.eye(12) - K @ self.H) @ self.P

        state = np.asarray(self.x).flatten().tolist()
        return np.array([[state[0], state[2], state[4]], [state[6], state[8], state[10]]])

        

class KF_pose_estimation(object):
    """Kalman filter for circle pose detection"""
    def __init__(self, dt, u_x, u_y, u_z, u_nx, u_ny, u_nz, std_acc, x_std_meas, y_std_meas, z_std_meas, nx_std_meas, ny_std_meas, nz_std_meas):
        """
            dt: sampling time (time for 1 cycle)
            u_x: acceleration in x-direction
            u_y: acceleration in y-direction
            u_z: acceleration in z-direction
            u_nx: acceleration in normal vector x-direction
            u_ny: acceleration in normal vector y-direction
            u_nz: acceleration in normal vector z-direction
            std_acc: process noise magnitude
            x_std_meas: standard deviation of the measurement in x-direction
            y_std_meas: standard deviation of the measurement in y-direction
            z_std_meas: standard deviation of the measurement in z-direction
            nx_std_meas: standard deviation of the measurement in normal vector x-direction
            ny_std_meas: standard deviation of the measurement in normal vector y-direction
            nz_std_meas: standard deviation of the measurement in normal vector z-direction

            Consider the acceleration as process noise
        """
        self.dt = dt
        self.x = np.matrix([[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]])   # Inital state
        self.u = np.matrix([[u_x],[u_y],[u_z],[u_nx],[u_ny],[u_nz]])            # Control input variable

        # State Transition Matrix F
        self.F = np.matrix([[1, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 1, dt, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 1, dt, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 1, dt, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 1, dt, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, dt],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])

        # Control Input Matrix B 
        self.B = np.matrix([[(dt**2)/2,0,0,0,0,0],
                            [dt,0,0,0,0,0],
                            [0,(dt**2)/2,0,0,0,0],
                            [0,dt,0,0,0,0],
                            [0,0,(dt**2)/2,0,0,0],
                            [0,0,dt,0,0,0],
                            [0,0,0,(dt**2)/2,0,0],
                            [0,0,0,dt,0,0],
                            [0,0,0,0,(dt**2)/2,0],
                            [0,0,0,0,dt,0],
                            [0,0,0,0,0,(dt**2)/2],
                            [0,0,0,0,0,dt]])
        
        # Measurement Mapping Matrix H
        self.H = np.matrix([[1,0,0,0,0,0,0,0,0,0,0,0],
                            [0,0,1,0,0,0,0,0,0,0,0,0],
                            [0,0,0,0,1,0,0,0,0,0,0,0],
                            [0,0,0,0,0,0,1,0,0,0,0,0],
                            [0,0,0,0,0,0,0,0,1,0,0,0],
                            [0,0,0,0,0,0,0,0,0,0,1,0]])

        # Initial Process Noise Covariance Q
        self.Q = np.matrix([[(dt**4)/4, (dt**3)/2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [(dt**3)/2, (dt**2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, (dt**4)/4, (dt**3)/2, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, (dt**3)/2, (dt**2), 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, (dt**4)/4, (dt**3)/2, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, (dt**3)/2, (dt**2), 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, (dt**4)/4, (dt**3)/2, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, (dt**3)/2, (dt**2), 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, (dt**4)/4, (dt**3)/2, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, (dt**3)/2, (dt**2), 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, (dt**4)/4, (dt**3)/2],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, (dt**3)/2, (dt**2)]] * (std_acc**2))

        # Initial Measurement Noise Covariance R
        self.R = np.matrix([[x_std_meas**2, 0, 0, 0, 0, 0],
                            [0, y_std_meas**2, 0, 0, 0, 0],
                            [0, 0, z_std_meas**2, 0, 0, 0],
                            [0, 0, 0, nx_std_meas**2, 0, 0],
                            [0, 0, 0, 0, ny_std_meas**2, 0],
                            [0, 0, 0, 0, 0, nz_std_meas**2]])

        # Initial Covariance Matrix P
        self.P = np.eye(self.F.shape[1])


    def predict(self):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, self.u)
        self.P = self.F @ self.P @ self.F.T + self.Q
        return np.array([self.x[0], self.x[2], self.x[4], self.x[6], self.x[8], self.x[10]])

    def update(self, z):
        y = z - self.H @ self.x
        self.S = self.H @ self.P @ self.H.T + self.R
        kalman_gain = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
        self.x = self.x + kalman_gain @ y
        self.P = (np.eye(self.P.shape[0],self.P.shape[1]) - kalman_gain @ self.H) @ self.P

        return np.array([self.x[0], self.x[2], self.x[4], self.x[6], self.x[8], self.x[10]])



class KF_ellipse_detection(object):
    """Kalman filter for ellipse detection"""
    def __init__(self, dt, u_x, u_y, u_a, u_b, u_theta, std_acc, x_std_meas, y_std_meas, a_std_meas, b_std_meas, theta_std_meas):
        """
            dt: sampling time (time for 1 cycle)
            u_x: acceleration in x-direction
            u_y: acceleration in y-direction
            u_a: acceleration in major axis length
            u_b: acceleration in minor axis length
            u_theta: acceleration in angle
            std_acc: process noise magnitude
            x_std_meas: standard deviation of the measurement in x-direction
            y_std_meas: standard deviation of the measurement in y-direction
            a_std_meas: standard deviation of the measurement in major axis length
            b_std_meas: standard deviation of the measurement in minor axis length
            theta_std_meas: standard deviation of the measurement in angle
        """
        # State:
        # [x, v_x, y, v_y, a, v_a, b, v_b, theta, v_theta]
        self.dt = dt # sampling time
        self.u = np.matrix([[u_x],[u_y],[u_a],[u_b], [u_theta]]) # control input variables
        self.x = np.matrix([[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]]) # intial State
        
        # State Transition Matrix F
        self.F = np.matrix([[1, dt, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 1, dt, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 1, dt, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 1, dt, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 1, dt],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1],])

        # Control Input Matrix B 
        self.B = np.matrix([[(self.dt**2)/2, 0, 0, 0, 0],
                            [self.dt, 0, 0, 0, 0],
                            [0, (self.dt**2)/2, 0, 0, 0],
                            [0, self.dt, 0, 0, 0],
                            [0, 0, (self.dt**2)/2, 0, 0],
                            [0, 0, self.dt, 0, 0],
                            [0, 0, 0, (self.dt**2)/2, 0],
                            [0, 0, 0, self.dt, 0],
                            [0, 0, 0, 0, (self.dt**2)/2],
                            [0, 0, 0, 0, self.dt],])

        # Measurement Mapping Matrix 
        self.H = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 1, 0]])

        # Initial Process Noise Covariance
        self.Q = np.matrix([[(dt**4)/4, (dt**3)/2, 0, 0, 0, 0, 0, 0, 0, 0],
                            [(dt**3)/2, (dt**2), 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, (dt**4)/4, (dt**3)/2, 0, 0, 0, 0, 0, 0],
                            [0, 0, (dt**3)/2, (dt**2), 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, (dt**4)/4, (dt**3)/2, 0, 0, 0, 0],
                            [0, 0, 0, 0, (dt**3)/2, (dt**2), 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, (dt**4)/4, (dt**3)/2, 0, 0],
                            [0, 0, 0, 0, 0, 0, (dt**3)/2, (dt**2), 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, (dt**4)/4, (dt**3)/2],
                            [0, 0, 0, 0, 0, 0, 0, 0, (dt**3)/2, (dt**2)]] * (std_acc**2))

        #Initial Measurement Noise Covariance (complete the definition)
        self.R = np.matrix([[x_std_meas**2, 0, 0, 0, 0],
                            [0, y_std_meas**2, 0, 0, 0],
                            [0, 0, a_std_meas**2, 0, 0],
                            [0, 0, 0, b_std_meas**2, 0],
                            [0, 0, 0, 0, theta_std_meas**2]])

        #Initial Covariance Matrix
        self.P = np.eye(self.F.shape[1])

    def predict(self):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, self.u)
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        state = np.asarray(self.x).flatten().tolist()
        return [(state[0], state[2]), (state[4], state[6]), state[8]]

    def update(self, measurement):
        z = np.array([[measurement[0][0]], 
                      [measurement[0][1]], 
                      [measurement[1][0]], 
                      [measurement[1][1]], 
                      [measurement[2]]])
        
        y = z - self.H @ self.x
        self.S = self.H @ self.P @ self.H.T + self.R
        kalman_gain = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
        self.x = self.x + kalman_gain @ y
        self.P = (np.eye(self.P.shape[0],self.P.shape[1]) - kalman_gain @ self.H) @ self.P
        
        state = np.asarray(self.x).flatten().tolist()
        return [(state[0], state[2]), (state[4], state[6]), state[8]]