import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
import cv2

class TurtleBot4Controller(Node):
    def __init__(self):

        super().__init__('turtlebot4_controller')
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.bridge = CvBridge()

    def depth_callback(self, depth_msg):
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg)

        # Extract a region of interest (ROI) from the depth image
        height, width = depth_image.shape
        roi = depth_image[int(height/2) - 50:int(height/2) + 50, int(width/2) - 50:int(width/2) + 50]

        # Calculate the mean depth value in the ROI
        mean_depth = np.nanmean(roi)

        twist_msg = Twist()
        if np.isnan(mean_depth) or mean_depth > 1.0:  # 1.0 meters threshold
            twist_msg.linear.x = 0.5
        else:
            twist_msg.linear.x = 0

        self.publisher.publish(twist_msg)
        self.get_logger().info('Mean depth: %f, Publishing cmd_vel: linear=%f' % (mean_depth, twist_msg.linear.x))

def main(args=None):
    try:
        rclpy.init(args=args)
        tb4_controller = TurtleBot4Controller()
        rclpy.spin(tb4_controller)
    except KeyboardInterrupt:
        print('TurtleBot4 controller interrupted by user. Shutting down...')
    finally:
        if tb4_controller:
            tb4_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
