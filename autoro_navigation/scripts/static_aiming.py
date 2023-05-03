#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

import numpy as np
from simple_pid import PID

class StaticAiming(Node):
    def __init__(self):
        super().__init__('static_aiming')

        # Robot will search by rotating slowly until a target is found. Once a target message appears, the mode will switch to 'aiming'
        self.mode = 'search'
        self.target_x = 0
        self.target_y = 0
        self.target_z = 0

        self.target_pose_subscription = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.target_pose_callback,
            10)
        self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.pid = PID(1, 0.1, 0.05, setpoint = 0)
        self.pid.sample_time = 0.01
        
    def target_pose_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_z = msg.z
        self.mode = 'search'

    # Inputs: target x,y,z pose
    # Outputs: azimuth and elevation of target
    def calculate_angles(self):
        azimuth = np.rad2deg(np.arctan(self.target_x / self.target_z))
        elevation = np.rad2deg(np.arctan(-self.target_y / self.target_z))
        return azimuth, elevation
    
    def search_action(self):
        while(self.mode == 'search'):
            twist_msg = Twist()
            linear_speed = 0
            angular_speed = 1

            twist_msg.linear.x = linear_speed
            twist_msg.angular.z = angular_speed

            self.twist_publisher.publish(twist_msg)

    def horizontal_aim_action(self):
        twist_msg = Twist()
        azimuth, _ = self.calculate_angles()
        while azimuth > 5:
            angular_velocity = self.pid(azimuth)
            twist_msg.z = angular_velocity
            self.twist_publisher.publish(twist_msg)
            azimuth, _ = self.calculate_angles()

def main(args=None):
    rclpy.init(args=args)
    static_aiming = StaticAiming()
    rclpy.spin(static_aiming)

    static_aiming.search_action()
    static_aiming.horizontal_aim_action()

    static_aiming.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()