import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import GetCostmap

from geometry_msgs.msg import Point32, Polygon
import matplotlib.pyplot as plt

import numpy as np

class GetCostmapNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        self.get_costmap_client = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        while not self.get_costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service "/global_costmap/get_costmap" not available, waiting...')

    def inflate_costmap(self):
        # Call the "/global_costmap/get_costmap" service to get the current costmap
        request = GetCostmap.Request()
        future = self.get_costmap_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            costmap = future.result().map
            print(costmap.GetSizeInCellsX())
        else:
            self.get_logger().error('Failed to get costmap: %r' % future.exception())
            return
        


def main(args=None):
    rclpy.init(args=args)
    node = GetCostmapNode()
    node.inflate_costmap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()