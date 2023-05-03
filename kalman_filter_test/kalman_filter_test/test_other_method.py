import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np


camera_matrix = np.array([ [196.7876739501953, 0.0, 123.86207580566406],
                            [0.0, 196.7876739501953, 127.05023193359375],
                            [0.0, 0.0, 1.0]]) * 10

def create_circle_model_points(radius, num_points=8):
    angle_step = 2 * np.pi / num_points
    model_points = [(0, 0, 0)]

    for i in range(num_points):
        x = radius * np.cos(i * angle_step)
        y = radius * np.sin(i * angle_step)
        model_points.append((x, y, 0))

    return np.array(model_points, dtype=np.float32)

def estimate_pose(model_points, image_points, camera_matrix, dist_coeffs=None):
    if dist_coeffs is None:
        dist_coeffs = np.zeros((4, 1))

    _, rvec, tvec = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs)

    return rvec, tvec

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.color_subscription = self.create_subscription(
            Image,
            '/color/preview/image',
            self.color_listener_callback,
            10)
        self.depth_subscription = self.create_subscription(
            Image,
            '/stereo/depth',
            self.depth_listener_callback,
            10)

        self.bridge = CvBridge()

        self.target_point_pub = self.create_publisher(PointStamped, '/target_point', 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)

        self.color_image = None
        self.depth_image = None

    def color_listener_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_images()

    def depth_listener_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        self.process_images()

    def process_images(self):
        if self.color_image is None or self.depth_image is None:
            return

        target_data = self.find_target_circle(self.color_image)

        if target_data:
            position, pose = target_data

            # Publish position to /target_point topic
            target_point_msg = PointStamped()
            target_point_msg.header.stamp = self.get_clock().now().to_msg()
            target_point_msg.header.frame_id = 'camera_link'
            target_point_msg.point.x, target_point_msg.point.y, target_point_msg.point.z = position
            self.target_point_pub.publish(target_point_msg)

            # Publish pose to /target_pose topic
            target_pose_msg = PoseStamped()
            target_pose_msg.header.stamp = self.get_clock().now().to_msg()
            target_pose_msg.header.frame_id = 'camera_link'
            target_pose_msg.pose.orientation.w, target_pose_msg.pose.orientation.x, target_pose_msg.pose.orientation.y, target_pose_msg.pose.orientation.z = pose
            self.target_pose_pub.publish(target_pose_msg)

            print("published")

    def find_target_circle(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)

        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20,
                                   param1=100, param2=30,
                                   minRadius=10, maxRadius=100)

        if circles is not None:
            # Convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")

            # Assume the first circle is the target
            x, y, radius = circles[0]

            # Define the circle's 3D model points
            model_points = create_circle_model_points(radius)

            # Calculate the 2D image points of the target circle
            num_points = len(model_points)
            image_points = np.zeros((num_points, 2), dtype=np.float32)
            for i, model_point in enumerate(model_points):
                image_points[i, 0] = x + model_point[0]
                image_points[i, 1] = y + model_point[1]

            # Estimate the pose of the target circle
            rvec, tvec = estimate_pose(model_points, image_points, camera_matrix)

            # Convert the rotation vector to a quaternion
            _, quaternion = cv2.Rodrigues(rvec)
            quaternion = quaternion.flatten()

            pose = np.concatenate(([tvec[0, 0], tvec[1, 0], tvec[2, 0]], quaternion))

            # Calculate the real-world position of the circle
            depth = self.depth_image[y, x]
            position = np.array([0.0, 0.0, 0.0])

            if depth != 0:
                z = depth
                position[0] = (x - camera_matrix[0, 2]) * z / camera_matrix[0, 0]
                position[1] = (y - camera_matrix[1, 2]) * z / camera_matrix[1, 1]
                position[2] = z


            return position, pose

        return None


def main(args=None):
    rclpy.init(args=args)

    image_processor = ImageProcessor()

    rclpy.spin(image_processor)

    # Destroy the node explicitly
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
