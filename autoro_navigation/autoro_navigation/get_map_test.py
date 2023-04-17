import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.srv import GetMap

from geometry_msgs.msg import Point32, Polygon
import matplotlib.pyplot as plt

import numpy as np

class GetMapNode(Node):
    def __init__(self):
        super().__init__('get_map_node')

        self.map_ = None            # 2D array with occupation probability 0-255
        self.occupancy_grid_ = None     # 2D array with occupation = 1 if costmap_ > occupied threshold
        self.inflated_grid_ = None      # occupancy_grid_ inflated to account for robot radius


        self.occupied_threshold_ = 0.80

        self.get_map_client_ = self.create_client(GetMap, '/map_server/map')
        while not self.get_map_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service "/map_server/map" not available, waiting...')

    def update_map(self):
        # Call the "/map_server/map" service to get the current costmap
        request = GetMap.Request()
        future = self.get_map_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            map = future.result().map
        else:
            self.get_logger().error('Failed to get map: %r' % future.exception())
            return
        
        self.map_ = np.array(map.data).reshape((map.info.height, map.info.width))


        self.visualize_map()

        
    def visualize_map(self):
        fig, ax = plt.subplots()
        img = ax.imshow(self.map_, cmap='coolwarm')
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = GetMapNode()
    node.update_map()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()