import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from kalman_filter_test.KF import *


from kalman_filter_test.target_filter import *
from kalman_filter_test.target_pose_estimation import *

class circle_tracker(Node):

    def __init__(self):
        # inherit from Node
        super().__init__('circle_tracker')

        # create subscription
        # self.subscription = self.create_subscription(Image, '/color/preview/image', self.sub_callback, 10)
        self.subscription = self.create_subscription(Image, '/oakd_lite_camera/rgb_image', self.sub_callback, 10)

        self.publisher = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.point_pub = self.create_publisher(Point, '/target_point', 10)


        # make OpenCV bridge
        self.bridge = CvBridge()

        # method variables
        self.K = np.array([ [196.7876739501953, 0.0, 123.86207580566406],
                            [0.0, 196.7876739501953, 127.05023193359375],
                            [0.0, 0.0, 1.0]]) * 10
        self.K[2,2] = 1
        
        # old K
        # self.K = np.array([[8.982687231767234834e+02,0.000000000000000000e+00,5.549473975536263879e+02],
        #             [0.000000000000000000e+00,8.893338438389678231e+02,3.154322774257576043e+02],
        #             [0.000000000000000000e+00,0.000000000000000000e+00,1.000000000000000000e+00]])

        # this might be the distrotion matrix from the camera/info?
        # self.distortion = np.array([-4.0933966636657715, 9.190781593322754, 0.0012543922057375312, -0.0010304413735866547, -8.917245864868164, -4.187956809997559, 9.556831359863281, -9.303533554077148])
        self.distortion = None

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
        
        self.kalman_ellipse = KF_ellipse_detection(dt=0.1,
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

        self.extended_kalman_pose = EKF_pose_estimation(dt=0.1,
                                               std_acc=1,
                                               x_std_meas=.05,
                                               y_std_meas=.05,
                                               z_std_meas=.05,
                                               nx_std_meas=.05,
                                               ny_std_meas=.05,
                                               nz_std_meas=.05)
        
        self.u_x = 1; self.u_y = 1; self.u_z = 1
        self.u_nx = 1; self.u_ny = 1; self.u_nz = 1


    def sub_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        pose_msg = PoseStamped()

                # Undistort Image using intrinsic parameters
        undistorted_frame = cv2.undistort(cv_image, self.K, self.distortion)

        # Filter image for target
        filtered_frame = filter_by_color(undistorted_frame)
        # filtered_frame = filter_by_brightness(undistorted_frame)

        # Detect contours of target
        frame_w_contours, contours, heirarchy = find_contours_connected_regions(filtered_frame)

        # Calculate best fitting ellipse from contours
        ellipse_w_max_area, max_area = find_target(cv_image, contours, heirarchy)

        image_w_ellipse = cv_image
        # If ellipse is found, calculate target pose
        if np.all(ellipse_w_max_area != None):
            # Filter ellipse through Kalman filter

            out_Point = Point()
            out_Point.x = ellipse_w_max_area[0][0]
            out_Point.y = ellipse_w_max_area[0][1]
            out_Point.z = 0.0

            self.point_pub.publish(out_Point)



            self. kalman_ellipse.predict()
            best_ellipse = self.kalman_ellipse.update(ellipse_w_max_area)

            # Filter circle pose through extended Kalman filter
            observed_target_pose = determine_target_pose(self.K, best_ellipse)
            # print(observed_target_pose)
            
            # # Get movement input
            v_x = 0
            v_y = 0
            w_z = 0

            u = [self.u_x, self.u_y, self.u_z, self.u_nx, self.u_ny, self.u_nz, v_x, v_y, w_z]    # input
            z = np.array([observed_target_pose[0].flatten()]).T     # measurement
            
            filter_target_pose = self.extended_kalman_pose.filter(u, z).flatten()
            # print(filter_target_pose)

            pose_msg.pose.position.x = filter_target_pose[0]
            pose_msg.pose.position.y = filter_target_pose[1]
            pose_msg.pose.position.z = filter_target_pose[2]
            pose_msg.pose.orientation.x = filter_target_pose[3]
            pose_msg.pose.orientation.y = filter_target_pose[4]
            pose_msg.pose.orientation.z = filter_target_pose[5]
            self.publisher.publish(pose_msg)


            # Filter circle pose through Kalman filter
            # kalman_pose.predict()
            # observed_target_pose = determine_target_pose(K, best_ellipse)
            # filter_target_pose = kalman_pose.update(observed_target_pose[0].flatten().T).flatten()

            # Plot 2D visualization of ellipse norm (plot resets after 500 timesteps)
            # count += 1
            # plot_circle_norm_2d(target_pose[:][1], fig_2d, ax_2d, count)
            
            # Plot 3D visualization of ellipse norm
            plot_circle_norm_3d(filter_target_pose, self.ax_3d)
            # plot_circle_norm_3d_both_solutions(filter_target_pose, ax_3d)

            # Visualize best fitting ellipse
            image_w_ellipse = cv2.ellipse(cv_image, ellipse_w_max_area, (0,255,0), 2)
            if best_ellipse[1][0] >= 0 and best_ellipse[1][1] >= 0:
                image_w_ellipse = cv2.ellipse(cv_image, best_ellipse, (255,0,0), 2)
            else:
                print("Invalid ellipse dimensions:", best_ellipse)

            # image_w_ellipse = cv2.ellipse(cv_image, best_ellipse, (255,0,0), 2)        

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
