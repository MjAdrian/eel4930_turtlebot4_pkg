#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

import numpy as np
from simple_pid import PID
from autoro_interfaces.srv import YAng, Trig

class Shoot(Node):
    def __init__(self):
        super().__init__('shoot_aimed')

        self.y_aimed = self.create_subscription(
            Bool,
            '/y_aimed',
            self.y_callback,
            1)
        self.x_aimed = self.create_subscription(
            Bool,
            '/x_aimed',
            self.x_callback,
            1)
        
        self.cli = self.create_client(Trig, 'Trig')

        self.req = Trig.Request()
        self.req.time = 0.0

        self.future = self.cli.call_async(self.req)

        self.req.time = .75

        self.x = False
        self.y = False
        

    def x_callback(self, msg: Bool):
        self.x = msg.data

        if self.x and self.y:
            self.future = self.cli.call_async(self.req)

    def y_callback(self, msg: Bool):
        self.y = msg.data

        if self.x and self.y:
            self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    static_aiming = Shoot()

    while rclpy.ok():
        rclpy.spin_once(static_aiming)
        if static_aiming.future.done():
            try:
                response = static_aiming.future.result()
            except Exception as e:
                static_aiming.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                pass

    static_aiming.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()