import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import depthai as dai

class OakDLitePublisher(Node):
    def __init__(self):
        super().__init__('oakd_lite_publisher')

        # create publishers
        self.rgb_publisher_ = self.create_publisher(Image, '/oakd_lite_camera/rgb_image', 10)
        self.depth_publisher_ = self.create_publisher(Image, '/oakd_lite_camera/depth_image', 10)

        # set up CV Bridge
        self.bridge = CvBridge()

        # Make object
        self.pipeline = dai.Pipeline()

        # Set up colored images
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")

        # Set up depth images
        cam_mono = self.pipeline.create(dai.node.MonoCamera)
        cam_depth = self.pipeline.create(dai.node.StereoDepth)
        xout_depth = self.pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")

        cam_depth.setConfidenceThreshold(200)
        cam_depth.setMedianFilter(dai.StereoDepthProperties.MedianFilter.KERNEL_7x7)

        # set up mono camera
        cam_mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        # image properties
        cam_rgb.setPreviewSize(300, 300)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)

        # set up output link for RGB images
        cam_rgb.preview.link(xout_rgb.input)

        # set up output link for depth images
        cam_rgb.preview.link(cam_depth.left)
        cam_mono.out.link(cam_depth.right)
        cam_depth.depth.link(xout_depth.input)

        # set up device instance
        self.device = dai.Device(self.pipeline)
        self.q_rgb = self.device.getOutputQueue("rgb", 4, blocking=False)
        self.q_depth = self.device.getOutputQueue("depth", 4, blocking=False)

    def publish_camera_data(self):
        # get msg, convert to CV img msg then publish RGB images
        frame_rgb = self.q_rgb.get().getCvFrame()
        image_message_rgb = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="bgr8")
        self.rgb_publisher_.publish(image_message_rgb)

        # get msg, convert to CV img msg then publish depth images
        frame_depth = self.q_depth.get().getCvFrame()
        image_message_depth = self.bridge.cv2_to_imgmsg(frame_depth, encoding="mono16")
        self.depth_publisher_.publish(image_message_depth)

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
