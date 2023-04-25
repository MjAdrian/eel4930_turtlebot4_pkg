import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import depthai as dai

class OakDLitePublisher(Node):
    def __init__(self):
        super().__init__('oakd_lite_publisher')
        self.publisher_ = self.create_publisher(Image, '/oakd_lite_camera/rgb_image', 10)
        self.bridge = CvBridge()

        self.init_camera()

    def init_camera(self):
        self.pipeline = dai.Pipeline()

        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")

        cam_rgb.setPreviewSize(300, 300)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)

        cam_rgb.preview.link(xout_rgb.input)

        self.device = dai.Device(self.pipeline)
        self.q_rgb = self.device.getOutputQueue("rgb", 4, blocking=False)

    def publish_camera_data(self):
        frame_rgb = self.q_rgb.get().getCvFrame()
        image_message = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="bgr8")
        self.publisher_.publish(image_message)

def main(args=None):
    rclpy.init(args=args)
    oakd_lite_publisher = OakDLitePublisher()

    try:
        while rclpy.ok():
            rclpy.spin_once(oakd_lite_publisher, timeout_sec=0.1)
            oakd_lite_publisher.publish_camera_data()
    except KeyboardInterrupt:
        pass

    oakd_lite_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
