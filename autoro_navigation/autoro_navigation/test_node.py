import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.srv import GetCostmap
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped
import tf_transformations

import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

        self.nav_ = BasicNavigator()

        # All maps are stored as a 2D Numpy array
        self.map_ = None                # Laser scan map only, no inflation
        self.costmap_ = None            # Nav2 costmap with occupation probability
        self.binary_costmap_ = None     # Binary costmap with inflation

        self.width_ = 0                 # Width of maps
        self.height_ = 0                # Height of maps
        self.occupied_threshold_ = 0.97 # A cell is occupied if cost >= occupied_threshold
        self.coverage_threshold_ = 0.95 # Amount of the map the waypoint generator should attempt to cover

        self.search_waypoints = []      # List of waypoints robot needs to traverse to see all points on map

        # Client to request global map
        self.get_map_client_ = self.create_client(GetMap, '/map_server/map')
        while not self.get_map_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service "/map_server/map" not available, waiting...')

        # Client to request global costmap
        self.get_costmap_client = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        while not self.get_costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service "/global_costmap/get_costmap" not available, waiting...')

    def update_maps(self):
        # Call the "/map_server/map" service to get the current costmap
        request = GetMap.Request()
        future = self.get_map_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            map = future.result().map
        else:
            self.get_logger().error('Failed to get map: %r' % future.exception())
            return
        self.width_ = map.info.width
        self.height_ = map.info.height
        self.map_ = np.array(map.data).reshape((self.height_, self.width_))

        # Call the "/global_costmap/get_costmap" service to get the current costmap
        request = GetCostmap.Request()
        future = self.get_costmap_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            costmap = future.result().map
        else:
            self.get_logger().error('Failed to get costmap: %r' % future.exception())
            return
        self.costmap_ = np.array(costmap.data).reshape((self.height_, self.width_))

        # Create occupancy_grid_ based on costmap values
        self.binary_costmap_ = np.copy(self.costmap_)
        occupied_mask = self.binary_costmap_ >= (self.occupied_threshold_ * 255)
        self.binary_costmap_[occupied_mask] = 1
        self.binary_costmap_[~occupied_mask] = 0

        self.get_logger().info('All maps updated!')

    def generate_waypoints(self):
        # '0' for unseen cell (all empty cells right now), '100' for cells with obstacle
        unseen_map = np.copy(self.map_)

        # First clustering, keep the cluster that represents the interior (search) space
        labels, n_clusters, cluster_map = self.generate_clusters(unseen_map)

        # Update unseen_map to only include cluster '1' (this is assumed to be the search space
        # if there is an unvisited border around the edge of the map)
        mask = cluster_map == 1.0
        unseen_map[mask] = 0
        unseen_map[~mask] = 100

        self.visualize_map(unseen_map)

        ###### TODO #####

        # Randomly propose waypoints within unseen_map
        # Choose waypoints that can see the most unseen points
        # Recluster unseen points, repeat process on all clusters until coverage is acceptable

    def generate_clusters(self, map):
        # Create Numpty array containing coordinates of '0' cells
        empty_cells = []
        for x in range(self.width_):
            for y in range(self.height_):
                if map[y, x] == 0:
                    empty_cells.append([x, y])
        empty_cells = np.array(empty_cells)

        db = DBSCAN(eps=np.sqrt(2), min_samples=9).fit(empty_cells)
        labels = db.labels_   
        n_clusters = len(set(labels)) - (1 if -1 in labels else 0)   

        # Produce cluster map
        cluster_map = np.copy(map)
        for i in range(len(empty_cells)):
            nx, ny = empty_cells[i]
            cluster_map[ny, nx] = labels[i]
        print(n_clusters)
        return labels, n_clusters, cluster_map

    def visualize_map(self, map):
        fig, ax = plt.subplots()
        img = ax.imshow(map, cmap='coolwarm')
        plt.show()

    def set_initial_pose(self):
        # Set initial pose
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.nav_.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = -0.0
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.x = q_x
        initial_pose.pose.orientation.y = q_y
        initial_pose.pose.orientation.z = q_z
        initial_pose.pose.orientation.w = q_w
        self.nav_.setInitialPose(initial_pose)

        # Wait for Nav2
        self.nav_.waitUntilNav2Active()

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()

    node.set_initial_pose()
    node.update_maps()
    node.generate_waypoints()
    #node.visualize_map(node.map_)
    #node.visualize_map(node.occupancy_grid_)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()