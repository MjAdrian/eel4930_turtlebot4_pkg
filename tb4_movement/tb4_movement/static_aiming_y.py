#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

import numpy as np
from simple_pid import PID
from autoro_interfaces.srv import YAng, Trig

class StaticAiming(Node):
    def __init__(self):
        super().__init__('static_aiming')

        # Robot will search by rotating slowly until a target is found. Once a target message appears, the mode will switch to 'aiming'
        self.mode = 'search'
        self.target_x = 0
        self.target_y = 0
        self.target_z = 0

        self.target_pose_subscription = self.create_subscription(
            Point,
            '/target_point',
            self.target_pose_callback,
            10)
        #self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.y_cli = self.create_client(YAng, 'Aim_Y')
        self.y_ang = 0

        self.aimed = self.create_publisher(Bool, 'y_aimed', 1)
        self.req = YAng.Request()
        self.req.angle = 0.0
        self.future = self.y_cli.call_async(self.req)
        #self.pid = PID(0.1, 0.0, 0.05, setpoint = 0)
        #self.pid.sample_time = 0.01

    def target_pose_callback(self, msg: Point):
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_z = msg.z
        self.mode = 'found'
        azimuth, elevation = self.calculate_angles()

        if abs(elevation - self.y_ang) > 10:
            aimed = Bool()
            aimed.data = False
            self.aimed.publish(aimed)
            #req = self.YAng.Request()
            self.req.angle = float(elevation - self.y_ang)
            self.future = self.y_cli.call_async(self.req)
            
        else:
            aimed = Bool()
            aimed.data = True
            self.aimed.publish(aimed)




    # Inputs: target x,y,z pose
    # Outputs: azimuth and elevation of target
    def calculate_angles(self):
        azimuth = np.rad2deg(np.arctan((self.target_x - 320) / 122.8))
        elevation = np.rad2deg(np.arctan((-self.target_y + 240) / 174.4))
        print("azimuth:", azimuth)
        print("elevation:", elevation)
        return azimuth, elevation
    
    # def search_action(self):
    #     self.get_logger().info(self.mode)
    #     if(self.mode == 'search'):
    #         twist_msg = Twist()
    #         linear_speed = 0
    #         angular_speed = 1
    #         twist_msg.linear.x = linear_speed * 1.0
    #         twist_msg.angular.z = angular_speed * 0.2

    #         self.twist_publisher.publish(twist_msg)
    #     else:
    #         self.horizontal_aim_action()

    # def horizontal_aim_action(self):
    #     twist_msg = Twist()
    #     azimuth, _ = self.calculate_angles()
    #     while azimuth > 5:
    #         angular_velocity = -self.pid(azimuth)/25
    #         twist_msg.angular.z = angular_velocity
    #         self.twist_publisher.publish(twist_msg)
    #         azimuth, _ = self.calculate_angles()

def main(args=None):
    rclpy.init(args=args)
    static_aiming = StaticAiming()

    while rclpy.ok():
        rclpy.spin_once(static_aiming)
        if static_aiming.future.done():
            try:
                response = static_aiming.future.result()
            except Exception as e:
                static_aiming.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                static_aiming.y_ang = response.cur_ang

    static_aiming.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()