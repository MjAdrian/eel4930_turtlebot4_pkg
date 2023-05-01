import numpy as np
import cv2

from kalman_filter_test.target_filter import *
from kalman_filter_test.target_pose_estimation import *
from KF import KF_pose_estimation

# Find camera calibration   https://docs.opencv.org/3.4/dc/dbb/tutorial_py_calibration.html
# Pose Estimation           https://docs.opencv.org/3.4/d7/d53/tutorial_py_pose.html

def main():
    vid = cv2.VideoCapture(0)

    # count = 0
    # fig_2d, ax_2d = initialize_2d_plot()

    fig_3d, ax_3d = initalize_3d_plot()

    K = np.array([  [8.982687231767234834e+02,0.000000000000000000e+00,5.549473975536263879e+02],
                    [0.000000000000000000e+00,8.893338438389678231e+02,3.154322774257576043e+02],
                    [0.000000000000000000e+00,0.000000000000000000e+00,1.000000000000000000e+00]])
    distortion = None

    # Initalize Kalman Filter
    kalman_pose = KF_pose_estimation(dt=0.1,
                                     u_x=1, 
                                     u_y=1, 
                                     u_z=1, 
                                     u_nx=1, 
                                     u_ny=1, 
                                     u_nz=1, 
                                     std_acc=1,
                                     x_std_meas=1, 
                                     y_std_meas=1, 
                                     z_std_meas=1, 
                                     nx_std_meas=1, 
                                     ny_std_meas=1, 
                                     nz_std_meas=1)
    while True:
        ret, frame = vid.read()

        # Undistort Image using intrinsic parameters
        undistorted_frame = cv2.undistort(frame, K, distortion)

        # Filter image for target
        # filtered_frame = filter_by_color(undistorted_frame)
        filtered_frame = filter_by_brightness(undistorted_frame) # WIP

        # Detect contours of target
        frame_w_contours, contours, heirarchy = find_contours_connected_regions(filtered_frame)
        # canny_edges, contours, heirarchy = find_contours_canny(result)
        # _, contours, heirarchy = find_contours_adaptive_threshold(frame, result)

        # Calculate best fitting ellipse from contours
        image_w_ellipse, ellipse_w_max_area, max_area = find_target(frame, contours, heirarchy)

        # If ellipse is found, calculate target pose
        if np.all(ellipse_w_max_area != None):
            kalman_pose.predict()
            observed_target_pose = determine_target_pose(K,ellipse_w_max_area)
            filter_target_pose = kalman_pose.update(np.reshape(observed_target_pose[0].flatten(), (1, -1)).T)
            filter_target_pose = filter_target_pose.flatten()
            print(filter_target_pose)

            # Plot 2D visualization of ellipse norm (plot resets after 500 timesteps)
            # count += 1
            # plot_circle_norm_2d(target_pose[:][1], fig_2d, ax_2d, count)
            
            # Plot 3D visualization of ellipse norm
            plot_circle_norm_3d(filter_target_pose, ax_3d)

        cv2.imshow('frame', frame)
        cv2.imshow('filtered_frame', filtered_frame)
        cv2.imshow('image_w_ellipse', image_w_ellipse)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    vid.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()