#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

import numpy as np

class StaticAiming(Node):
    def __init__(self):
        super().__init__('static_aiming')

        # Robot will search by rotating slowly until a target is found. Once a target message appears, the mode will switch to 'aiming'
        self.mode = 'search'
        self.target_point = None
        self.prev_target_point = None

        # PD control parameters
        self.P = 0.05
        self.D = 0.001

        self.target_pose_subscription = self.create_subscription(
            Point,
            '/target_point',
            self.target_pose_callback,
            10)
        self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    # Aiming control happens here
    def target_pose_callback(self, msg: Point):
        self.prev_target_point = self.target_point
        self.target_point = msg
        azimuth, elevation = self.calculate_angles(self.target_point)
        prev_azimuth, prev_elevation = self.calculate_angles(self.prev_target_point)

        # If current and previous points are in similar location (reject random noise)
        if np.abs(prev_azimuth - azimuth) < 10:
            # If error is sufficiently high (more than 10 degrees off)
            if np.abs(azimuth) > 10:
                # Control movement (Sort of PD control)
                control = - self.P*azimuth + self.D*(prev_azimuth - azimuth)
                self.sendTwist(control)

        self.sendElevation(elevation)

    # Inputs: target x,y,z pose
    # Outputs: azimuth and elevation of target
    def calculate_angles(self, point: Point):
        azimuth = np.rad2deg(np.arctan((point.x - 320) / 122.8))
        elevation = np.rad2deg(np.arctan((-point.y + 240) / 174.4))
        self._logger.info("azimuth: {}".format(azimuth))
        self._logger.info("elevation: {}".format(elevation))
        return azimuth, elevation
    
    def sendTwist(self, angular_velocity):
        twist_msg = Twist()
        twist_msg.angular.z = angular_velocity
        self.twist_publisher.publish(twist_msg)

    def sendElevation(self, elevation_angle):
        # Send gun elevation angle command here
        return

def main(args=None):
    rclpy.init(args=args)
    static_aiming = StaticAiming()

    try:
        rclpy.spin(static_aiming)
    except KeyboardInterrupt:
        print("Exiting Program")
    finally:
        static_aiming.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()