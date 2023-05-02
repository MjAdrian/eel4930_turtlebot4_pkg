import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from kalman_filter_test.KF import KF_pose_estimation


from kalman_filter_test.target_filter import *
from kalman_filter_test.target_pose_estimation import *

class circle_tracker(Node):

    def __init__(self):
        # inherit from Node
        super().__init__('circle_tracker')

        # create subscription
        # self.subscription = self.create_subscription(Image, '/color/preview/image', self.sub_callback, 10)
        self.subscription = self.create_subscription(Image, '/oakd_lite_camera/rgb_image', self.sub_callback, 10)

        # make OpenCV bridge
        self.bridge = CvBridge()

        # method variables
        self.K = np.array([ [196.7876739501953, 0.0, 123.86207580566406],
                            [0.0, 196.7876739501953, 127.05023193359375],
                            [0.0, 0.0, 1.0]])
        
        # old K
        # self.K = np.array([[8.982687231767234834e+02,0.000000000000000000e+00,5.549473975536263879e+02],
        #             [0.000000000000000000e+00,8.893338438389678231e+02,3.154322774257576043e+02],
        #             [0.000000000000000000e+00,0.000000000000000000e+00,1.000000000000000000e+00]])

        # this might be the distrotion matrix from the camera/info?
        self.distortion = np.array([-4.0933966636657715, 9.190781593322754, 0.0012543922057375312, -0.0010304413735866547, -8.917245864868164, -4.187956809997559, 9.556831359863281, -9.303533554077148])
        # self.distortion = None

        # continue flag for waitKey()
        self.continue_flag = True

        # plot
        self.fig_3d, self.ax_3d = initalize_3d_plot()

        # Initalize Kalman Filter
        self.kalman_pose = KF_pose_estimation(dt=0.1,
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


    def sub_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Undistort Image using intrinsic parameters
        undistorted_frame = cv2.undistort(cv_image, self.K, self.distortion)

        # Filter image for target
        # filtered_frame = filter_by_color(undistorted_frame)
        filtered_frame = filter_by_brightness(undistorted_frame) # WIP

        # Detect contours of target
        frame_w_contours, contours, heirarchy = find_contours_connected_regions(filtered_frame)
        # canny_edges, contours, heirarchy = find_contours_canny(result)
        # _, contours, heirarchy = find_contours_adaptive_threshold(frame, result)

        # Calculate best fitting ellipse from contours
        image_w_ellipse, ellipse_w_max_area, max_area = find_target(cv_image, contours, heirarchy)

        # If ellipse is found, calculate target pose
        if np.all(ellipse_w_max_area != None):
            self.kalman_pose.predict()
            observed_target_pose = determine_target_pose(self.K, ellipse_w_max_area)
            filter_target_pose = self.kalman_pose.update(np.reshape(observed_target_pose[0].flatten(), (1, -1)).T)
            filter_target_pose = filter_target_pose.flatten()
            # print(filter_target_pose)

            # Plot 2D visualization of ellipse norm (plot resets after 500 timesteps)
            # count += 1
            # plot_circle_norm_2d(target_pose[:][1], fig_2d, ax_2d, count)
            
            # Plot 3D visualization of ellipse norm
            plot_circle_norm_3d(filter_target_pose, self.ax_3d)

        cv2.imshow('frame', cv_image)
        cv2.imshow('filtered_frame', filtered_frame)
        cv2.imshow('image_w_ellipse', image_w_ellipse)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.continue_flag = False


def main(args=None):
    rclpy.init(args=args)

    circle_tracker_obj = circle_tracker()

    try:
        while rclpy.ok() and circle_tracker_obj.continue_flag:
            rclpy.spin_once(circle_tracker_obj)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt has occured!")
    finally:
        circle_tracker_obj.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
