"""
    Author: Aditya Ramesh
    Contains main function to detect and estimate pose of the target
"""

import numpy as np
import cv2

from target_filter import *
from target_pose_estimation import *
from KF import EKF_pose_estimation, KF_pose_estimation, KF_ellipse_detection

# Find camera calibration   https://docs.opencv.org/3.4/dc/dbb/tutorial_py_calibration.html
# Pose Estimation           https://docs.opencv.org/3.4/d7/d53/tutorial_py_pose.html

if __name__ == '__main__':
    vid = cv2.VideoCapture(0)

    # count = 0
    # fig_2d, ax_2d = initialize_2d_plot()

    fig_3d, ax_3d = initalize_3d_plot()

    K = np.array([  [8.982687231767234834e+02,0.000000000000000000e+00,5.549473975536263879e+02],
                    [0.000000000000000000e+00,8.893338438389678231e+02,3.154322774257576043e+02],
                    [0.000000000000000000e+00,0.000000000000000000e+00,1.000000000000000000e+00]])
    distortion = None

    # Initalize Kalman Filter
    kalman_ellipse = KF_ellipse_detection(dt=0.1,
                                          u_x=1,
                                          u_y=1,
                                          u_a=1,
                                          u_b=1,
                                          u_theta=1,
                                          std_acc=1,
                                          x_std_meas=.1,
                                          y_std_meas=.1,
                                          a_std_meas=.1,
                                          b_std_meas=.1,
                                          theta_std_meas=.01)
    
    u_x = 1; u_y = 1; u_z = 1
    u_nx = 1; u_ny = 1; u_nz = 1
    extended_kalman_pose = EKF_pose_estimation(dt=0.1,
                                               std_acc=1,
                                               x_std_meas=.05,
                                               y_std_meas=.05,
                                               z_std_meas=.05,
                                               nx_std_meas=.05,
                                               ny_std_meas=.05,
                                               nz_std_meas=.05)
    
    # kalman_pose = KF_pose_estimation(dt=0.1,
    #                                  u_x=1, 
    #                                  u_y=1, 
    #                                  u_z=1, 
    #                                  u_nx=1, 
    #                                  u_ny=1, 
    #                                  u_nz=1, 
    #                                  std_acc=1,
    #                                  x_std_meas=1, 
    #                                  y_std_meas=1, 
    #                                  z_std_meas=1, 
    #                                  nx_std_meas=1, 
    #                                  ny_std_meas=1, 
    #                                  nz_std_meas=1)
    while True:
        ret, frame = vid.read()

        # Undistort Image using intrinsic parameters
        undistorted_frame = cv2.undistort(frame, K, distortion)

        # Filter image for target
        # filtered_frame = filter_by_color(undistorted_frame)
        filtered_frame = filter_by_brightness(undistorted_frame)

        # Detect contours of target
        frame_w_contours, contours, heirarchy = find_contours_connected_regions(filtered_frame)

        # Calculate best fitting ellipse from contours
        ellipse_w_max_area, max_area = find_target(frame, contours, heirarchy)

        image_w_ellipse = frame
        # If ellipse is found, calculate target pose
        if np.all(ellipse_w_max_area != None):
            # Filter ellipse through Kalman filter
            kalman_ellipse.predict()
            best_ellipse = kalman_ellipse.update(ellipse_w_max_area)

            # Filter circle pose through extended Kalman filter
            observed_target_pose = determine_target_pose(K, best_ellipse)
            print(observed_target_pose)
            
            # # Get movement input
            v_x = 0
            v_y = 0
            w_z = 10

            u = [u_x, u_y, u_z, u_nx, u_ny, u_nz, v_x, v_y, w_z]    # input
            z = np.array([observed_target_pose[0].flatten()]).T     # measurement
            
            filter_target_pose = extended_kalman_pose.filter(u, z).flatten()
            print(filter_target_pose)

            # Filter circle pose through Kalman filter
            # kalman_pose.predict()
            # observed_target_pose = determine_target_pose(K, best_ellipse)
            # filter_target_pose = kalman_pose.update(observed_target_pose[0].flatten().T).flatten()

            # Plot 2D visualization of ellipse norm (plot resets after 500 timesteps)
            # count += 1
            # plot_circle_norm_2d(target_pose[:][1], fig_2d, ax_2d, count)
            
            # Plot 3D visualization of ellipse norm
            plot_circle_norm_3d(filter_target_pose, ax_3d)
            # plot_circle_norm_3d_both_solutions(filter_target_pose, ax_3d)

            # Visualize best fitting ellipse
            image_w_ellipse = cv2.ellipse(frame, ellipse_w_max_area, (0,255,0), 2)
            image_w_ellipse = cv2.ellipse(frame, best_ellipse, (255,0,0), 2)        

        cv2.imshow('frame', frame)
        cv2.imshow('filtered frame', filtered_frame)
        cv2.imshow('image w ellipses', image_w_ellipse)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    vid.release()
    cv2.destroyAllWindows()