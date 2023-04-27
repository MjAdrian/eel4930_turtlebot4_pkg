#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav_msgs.msg import OccupancyGrid

from geometry_msgs.msg import Point32, Polygon
import matplotlib.pyplot as plt

import numpy as np

class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.nav = BasicNavigator()

        self.robot_radius = 0.1

        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10
        )

        self.costmap_ = None
        self.waypoints_ = []

    def costmap_callback(self, msg):
        self.costmap_ = PyCostmap2D(msg)
        width = self.costmap_.getSizeInCellsX()
        height = self.costmap_.getSizeInCellsY()
        self.get_logger().info('"%.2f"' % width)
        self.get_logger().info('"%.2f"' % height)

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
