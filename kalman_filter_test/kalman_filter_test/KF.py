"""
EEL 4930/5934: Autonomous Robots
University Of Florida
"""

import numpy as np

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
                            [0, 0, 0, 0, 0, nz_std_meas**2],])

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



# class KF_ellipse_detection(object):
#     """Kalman filter for ellipse detection"""
#     def __init__(self, dt, u_x, u_y, u_a, u_b, u_theta):
#         """
#             dt: sampling time (time for 1 cycle)
#             u_x: acceleration in x-direction
#             u_y: acceleration in y-direction
#             u_a: acceleration in major axis length
#             u_b: acceleration in minor axis length
#             u_theta: acceleration in angle
#             std_acc: process noise magnitude
#             x_std_meas: standard deviation of the measurement in x-direction
#             y_std_meas: standard deviation of the measurement in y-direction
#             a_std_meas: standard deviation of the measurement in major axis length
#             b_std_meas: standard deviation of the measurement in minor axis length
#             theta_std_meas: standard deviation of the measurement in angle
#         """
#         # State:
#         # [x, v_x, y, v_y, a, v_a, b, v_b, theta, v_theta]
#         self.dt = dt # sampling time
#         self.u = np.matrix([[u_x],[u_y],[u_a],[u_b], [u_theta]]) # control input variables
#         self.x = np.matrix([[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]]) # intial State
        
        
#         # State Transition Matrix F
#         self.F = np.matrix([[1, dt, 0, 0, 0, 0, 0, 0, 0, 0],
#                             [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
#                             [0, 0, 1, dt, 0, 0, 0, 0, 0, 0],
#                             [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
#                             [0, 0, 0, 0, 1, dt, 0, 0, 0, 0],
#                             [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
#                             [0, 0, 0, 0, 0, 0, 1, dt, 0, 0],
#                             [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
#                             [0, 0, 0, 0, 0, 0, 0, 0, 1, dt],
#                             [0, 0, 0, 0, 0, 0, 0, 0, 0, 1],])

#         # Control Input Matrix B 
#         self.B = np.matrix([[(self.dt**2)/2, 0, 0, 0, 0],
#                             [self.dt, 0, 0, 0, 0],
#                             [0, (self.dt**2)/2, 0, 0, 0],
#                             [0, self.dt, 0, 0, 0],
#                             [0, 0, (self.dt**2)/2, 0, 0],
#                             [0, 0, self.dt, 0, 0],
#                             [0, 0, 0, (self.dt**2)/2, 0],
#                             [0, 0, 0, self.dt, 0],
#                             [0, 0, 0, 0, (self.dt**2)/2],
#                             [0, 0, 0, 0, self.dt],])

#         # Measurement Mapping Matrix 
#         self.H = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0],
#                             [0, 1, 0, 0, 0, 0, 0, 0],
#                             [0, 0, 0, 0, 1, 0, 0, 0],
#                             [0, 0, 0, 0, 0, 1, 0, 0]])

#         # Initial Process Noise Covariance
#         self.Q = np.matrix([[(self.dt**4)/4, 0, (self.dt**3)/2, 0, 0, 0, 0, 0],
#                             [0, (self.dt**4)/4, 0, (self.dt**3)/2, 0, 0, 0, 0],
#                             [(self.dt**3)/2, 0, self.dt**2, 0, 0, 0, 0, 0],
#                             [0, (self.dt**3)/2, 0, self.dt**2, 0, 0, 0, 0],
#                             [0, 0, 0, 0, (self.dt**4)/4, 0, (self.dt**3)/2, 0],
#                             [0, 0, 0, 0, 0, (self.dt**4)/4, 0, (self.dt**3)/2],
#                             [0, 0, 0, 0, (self.dt**3)/2, 0, self.dt**2, 0],
#                             [0, 0, 0, 0, 0, (self.dt**3)/2, 0, self.dt**2]]) * std_acc**2

#         #Initial Measurement Noise Covariance (complete the definition)
#         self.R = np.matrix([[x_std_meas**2, 0, 0, 0],
#                             [0, y_std_meas**2, 0, 0],
#                             [0, 0, w_std_meas**2, 0],
#                             [0, 0, 0, h_std_meas**2]])

#         #Initial Covariance Matrix (defined for you)
#         self.P = np.eye(self.F.shape[1])


#     def predict(self):
#         ## complete this function
#         # Update time state (self.x): x_k =Fx_(k-1) + Bu_(k-1) 
#         # Calculate error covariance (self.P): P= F*P*F' + Q 
#         self.x = np.dot(self.F, self.x) + np.dot(self.B, self.u)
#         self.P = self.F @ self.P @ self.F.T + self.Q
#         return self.x[0:2], self.x[4:6]


#     def update(self, z):
#         ## complete this function
#         # Calculate S = H*P*H'+R
#         # Calculate the Kalman Gain K = P * H'* inv(H*P*H'+R)
#         # Update self.x
#         # Update error covariance matrix self.P
#         y = z - self.H @ self.x
#         self.S = self.H @ self.P @ self.H.T + self.R
#         kalman_gain = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
#         self.x = self.x + kalman_gain @ y
#         self.P = (np.eye(self.P.shape[0],self.P.shape[1]) - kalman_gain @ self.H) @ self.P
#         return self.x[0:2], self.x[4:6]
        

# An Ellipse Feature Tracking Method based on the Kalman Filter 
# https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7408002 
class KF_ellipse_detection(object):
    """Kalman filter for ellipse detection"""

    def __init__(self, dt, u_x, u_y, u_a, u_b, u_theta, std_meas):
        """
            dt: sampling time (time for 1 cycle)
            u_x: acceleration in x-direction
            u_y: acceleration in y-direction
            u_a: acceleration in major axis length
            u_b: acceleration in minor axis length
            u_theta: acceleration in angle rotated

            Consider the acceleration as process noise
        """
        self.dt = dt
        # State: [x, y, v_x, v_y, a, v_a, b, v_b, theta, v_theta]
        self.x = np.matrix([0],[0],[0],[0],[0],[0],[0],[0],[0],[0]) # Initial State
        self.u = np.matrix([u_x],[u_y],[u_a],[u_b],[u_theta]) # Control input measurement

        # State transition matrix F
        self.F = np.matrix([[1, self.dt, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 1, self.dt, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 1, self.dt, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 1, self.dt, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 1, self.dt],
                            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]
                            ])

        # Control Input Matrix B 
        # self.B = 

        # Measurement Mapping Matrix H

        # Initial Process Noise Covariance Q
        self.Q = np.matrix([self.dt/2, 0, 0, 0, 0],
                           [0, self.dt/2, 0, 0, 0],
                           [0, 0, self.dt/2, 0, 0],
                           [0, 0, 0, self.dt/2, 0],
                           [0, 0, 0, 0, self.dt/2])

        # Initial Measurement Noise Covariance R


        # Initial Covariance Matrix P
        self.P = 1*np.eye(self.F.shape[1])
    
    


class KF_2D(object):
    def __init__(self, dt, u_x, u_y, std_acc, x_std_meas, y_std_meas):
        """
            dt: sampling time (time for 1 cycle)
            u_x: acceleration in x-direction
            u_y: acceleration in y-direction
            std_acc: process noise magnitude
            x_std_meas: standard deviation of the measurement in x-direction
            y_std_meas: standard deviation of the measurement in y-direction
        """
        self.dt = dt # sampling time
        self.u = np.matrix([[u_x],[u_y]]) # control input variables
        self.x = np.matrix([[0], [0], [0], [0]]) # intial State
        # State:
        # [x, y, v_x, v_y]
        
        # State Transition Matrix F(complete the definition)
        self.F = np.matrix([[1, 0, dt, 0],
                            [0, 1, 0, dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

        # Control Input Matrix B (defined for you)
        self.B = np.matrix([[(self.dt**2)/2, 0],
                            [0,(self.dt**2)/2],
                            [self.dt,0],
                            [0,self.dt]])

        # Measurement Mapping Matrix (complete the definition)
        self.H = np.matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0]])

        # Initial Process Noise Covariance (defined for you)
        self.Q = np.matrix([[(self.dt**4)/4, 0, (self.dt**3)/2, 0],
                            [0, (self.dt**4)/4, 0, (self.dt**3)/2],
                            [(self.dt**3)/2, 0, self.dt**2, 0],
                            [0, (self.dt**3)/2, 0, self.dt**2]]) * std_acc**2

        #Initial Measurement Noise Covariance
        self.R = np.matrix([[x_std_meas, 0],
                            [0, y_std_meas]])

        #Initial Covariance Matrix (defined for you)
        self.P = np.eye(self.F.shape[1])


    def predict(self):
        ## complete this function
        # Update time state (self.x): x_k =Fx_(k-1) + Bu_(k-1) 
        # Calculate error covariance (self.P): P= F*P*F' + Q 
        self.x = np.dot(self.F, self.x) + np.dot(self.B, self.u)
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[0:2]


    def update(self, z):
        ## complete this function
        # Calculate S = H*P*H'+R
        # Calculate the Kalman Gain K = P * H'* inv(H*P*H'+R)
        # Update self.x
        # Update error covariance matrix self.P
        y = z - self.H @ self.x
        self.S = self.H @ self.P @ self.H.T + self.R
        kalman_gain = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
        self.x = self.x + kalman_gain @ y
        self.P = (np.eye(self.P.shape[0],self.P.shape[1]) - kalman_gain @ self.H) @ self.P
        return self.x[0:2]

class KF_4D(object):
    def __init__(self, dt, u_x, u_y, u_w, u_h, std_acc, x_std_meas, y_std_meas, w_std_meas, h_std_meas):
        """
            dt: sampling time (time for 1 cycle)
            u_x: acceleration in x-direction
            u_y: acceleration in y-direction
            u_w: acceleration in width
            u_h: acceleration in heigh
            std_acc: process noise magnitude
            x_std_meas: standard deviation of the measurement in x-direction
            y_std_meas: standard deviation of the measurement in y-direction
            w_std_meas: standard deviation of the measurement in width
            h_std_meas: standard deviation of the measurement in height
        """
        self.dt = dt # sampling time
        self.u = np.matrix([[u_x],[u_y],[u_w],[u_h]]) # control input variables
        self.x = np.matrix([[0], [0], [0], [0], [0], [0], [0], [0]]) # intial State
        # State:
        # [x, y, v_x, v_y, w, h, v_w, v_h]
        
        # State Transition Matrix F
        self.F = np.matrix([[1, 0, dt, 0, 0, 0, 0, 0],
                            [0, 1, 0, dt, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0, 0, 0],
                            [0, 0, 0, 0, 1, 0, dt, 0],
                            [0, 0, 0, 0, 0, 1, 0, dt],
                            [0, 0, 0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 0, 0, 1]])

        # Control Input Matrix B 
        self.B = np.matrix([[(self.dt**2)/2, 0, 0, 0],
                            [0,(self.dt**2)/2, 0, 0],
                            [self.dt, 0, 0, 0],
                            [0,self.dt, 0, 0],
                            [0, 0, (self.dt**2)/2, 0],
                            [0, 0, 0, (self.dt**2)/2],
                            [0, 0, self.dt, 0],
                            [0, 0, 0,self.dt]])

        # Measurement Mapping Matrix 
        self.H = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0, 0, 0],
                            [0, 0, 0, 0, 1, 0, 0, 0],
                            [0, 0, 0, 0, 0, 1, 0, 0]])

        # Initial Process Noise Covariance
        self.Q = np.matrix([[(self.dt**4)/4, 0, (self.dt**3)/2, 0, 0, 0, 0, 0],
                            [0, (self.dt**4)/4, 0, (self.dt**3)/2, 0, 0, 0, 0],
                            [(self.dt**3)/2, 0, self.dt**2, 0, 0, 0, 0, 0],
                            [0, (self.dt**3)/2, 0, self.dt**2, 0, 0, 0, 0],
                            [0, 0, 0, 0, (self.dt**4)/4, 0, (self.dt**3)/2, 0],
                            [0, 0, 0, 0, 0, (self.dt**4)/4, 0, (self.dt**3)/2],
                            [0, 0, 0, 0, (self.dt**3)/2, 0, self.dt**2, 0],
                            [0, 0, 0, 0, 0, (self.dt**3)/2, 0, self.dt**2]]) * std_acc**2

        #Initial Measurement Noise Covariance (complete the definition)
        self.R = np.matrix([[x_std_meas, 0, 0, 0],
                            [0, y_std_meas, 0, 0],
                            [0, 0, w_std_meas, 0],
                            [0, 0, 0, h_std_meas]])

        #Initial Covariance Matrix (defined for you)
        self.P = np.eye(self.F.shape[1])


    def predict(self):
        ## complete this function
        # Update time state (self.x): x_k =Fx_(k-1) + Bu_(k-1) 
        # Calculate error covariance (self.P): P= F*P*F' + Q 
        self.x = np.dot(self.F, self.x) + np.dot(self.B, self.u)
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[0:2], self.x[4:6]


    def update(self, z):
        ## complete this function
        # Calculate S = H*P*H'+R
        # Calculate the Kalman Gain K = P * H'* inv(H*P*H'+R)
        # Update self.x
        # Update error covariance matrix self.P
        y = z - self.H @ self.x
        self.S = self.H @ self.P @ self.H.T + self.R
        kalman_gain = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
        self.x = self.x + kalman_gain @ y
        self.P = (np.eye(self.P.shape[0],self.P.shape[1]) - kalman_gain @ self.H) @ self.P
        return self.x[0:2], self.x[4:6]
