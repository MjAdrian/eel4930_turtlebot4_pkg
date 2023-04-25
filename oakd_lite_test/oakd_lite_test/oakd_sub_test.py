import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class OakDLiteSubscriber(Node):
    def __init__(self):
        super().__init__('oakd_lite_subscriber')
        self.subscription = self.create_subscription(
            Image, '/oakd_lite_camera/rgb_image', self.listener_callback, 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("OAK-D Lite Stream", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    oakd_lite_subscriber = OakDLiteSubscriber()

    try:
        rclpy.spin(oakd_lite_subscriber)
    except KeyboardInterrupt:
        pass

    oakd_lite_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
