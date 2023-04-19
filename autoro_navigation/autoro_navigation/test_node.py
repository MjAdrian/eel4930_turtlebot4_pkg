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
from bresenham import bresenham

# Support class, put somewhere else probably
class CameraPose():
    def __init__(self):
        self.x_ = 0
        self.y_ = 0
        self.theta_ = 0

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

        self.nav_ = BasicNavigator()

        # All maps are stored as a 2D Numpy array
        self.map_ = None                # Laser scan map only, no inflation, is_obstacle ? 100 : 0
        self.costmap_ = None            # Nav2 costmap with occupation probability 0-255
        self.binary_costmap_ = None     # Binary costmap with inflation (inflated_cell_value = 255)

        self.width_ = 0                 # Width of maps
        self.height_ = 0                # Height of maps
        self.occupied_threshold_ = 0.97 # A cell is occupied if cost >= occupied_threshold
        self.coverage_threshold_ = 0.95 # Amount of the map the waypoint generator should attempt to cover
        self.camera_fov_ = 170          # Camera field of view in degrees

        self.search_waypoints = []      # List of waypoints robot needs to traverse to see all points on map

        # Client to request global map
        self.get_map_client_ = self.create_client(GetMap, '/map_server/map')
        while not self.get_map_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service "/map_server/map" not available, waiting...')

        # Client to request global costmap
        self.get_costmap_client = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        while not self.get_costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service "/global_costmap/get_costmap" not available, waiting...')


    # Updates self.map_, self.costmap_, self.binary_costmap_
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


    # TODO: Generate waypoints for search path based on maps
    def generate_waypoints(self):
        # '0' for unseen cell (all empty cells right now), '100' for cells with obstacle
        unseen_map = np.copy(self.map_)

        # First clustering, keep the cluster that represents the interior (search) space
        # Cell value in cluster map corresponds to cluster label (0 to n_clusters-1, -1 if not in cluster)
        labels, n_clusters, cluster_map = self.generate_clusters(unseen_map)

        # Update unseen_map to only include cluster '1' (this is assumed to be the search space
        # if there is an unvisited border around the edge of the map)
        mask = cluster_map == 1.0
        unseen_map[mask] = 0
        unseen_map[~mask] = 100

        ##### Test pose #####
        # Sets two initial camera poses and visualizes the coverage area on map

        self.visualize_map(self.costmap_)
        self.visualize_map(self.binary_costmap_)
        self.visualize_map(cluster_map)
        self.visualize_map(unseen_map)

        initial_camera_pose = CameraPose()
        initial_camera_pose.x_ = 50
        initial_camera_pose.y_ = 50
        initial_camera_pose.theta_ = 0

        second_camera_pose = CameraPose()
        second_camera_pose.x_ = 75
        second_camera_pose.y_ = 80
        second_camera_pose.theta_ = 270
        

        visible_points = self.get_unique_visible_points(unseen_map, initial_camera_pose, 90)
        for point in visible_points:
            unseen_map[point[1], point[0]] = 20

        visible_points = self.get_unique_visible_points(unseen_map, second_camera_pose, 90)
        for point in visible_points:
            unseen_map[point[1], point[0]] = 40

        self.visualize_map(unseen_map)

        ##### End test pose #####

        ###### TODO #####
        # Randomly propose waypoints within binary_costmap_, can optimize number of points
        # Choose waypoints that can see the most unseen points
        # Recluster unseen points, repeat process on all clusters until coverage is acceptable
        # Path plan through proposed waypoints


    # Returns a list of uniquely visible points from provided camera pose and currently visible points
    def get_unique_visible_points(self, unseen_map, camera_pose: CameraPose, fov):

        visible_points = []

        camera_x = camera_pose.x_
        camera_y = camera_pose.y_
        theta_ccw = camera_pose.theta_ - (fov/2)
        theta_cw = camera_pose.theta_ + (fov/2)
        
        # Longest possible projected ray is along diagonal of map
        ray_length = np.sqrt(self.width_**2 + self.height_**2)

        # Iterate through angles in FOV in half degree increments
        # 0 degrees around Z is facing east
        for theta in np.linspace(theta_ccw, theta_cw, fov*2):
            ray_endpoint_x = (int)(np.cos(np.radians(theta)) * ray_length) + camera_x
            ray_endpoint_y = (int)(np.sin(np.radians(theta)) * ray_length) + camera_y

            ray = list(bresenham(camera_x, camera_y, ray_endpoint_x, ray_endpoint_y))

            # Iterate through each point in ray until obstacle is encountered
            n = 0
            n_ray_x, n_ray_y = ray[n]

            # Iterate through points on ray until running into an obstacle
            while self.map_[n_ray_y, n_ray_x] == 0 \
                    and n_ray_y < self.height_ and n_ray_y >= 0 \
                    and n_ray_x < self.width_ and n_ray_x >= 0:
                # If point in not already in local visible_points and is in global unseen map,
                # add it to the visible list
                if ~(ray[n] in visible_points) and unseen_map[n_ray_y, n_ray_x] == 0:         
                    visible_points.append(ray[n])
                n = n+1
                n_ray_x, n_ray_y = ray[n]

        return visible_points


    # Segments map into clusters
    # Search area of interest should be labelled cluster '1', assuming it's bounded by walls on all sides
    # Should remove "crevices" caused by missing walls from noise in laser scan, tune min_samples for this
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


    # Send initial pose to navigation stack
    # Blocks execution until pose set successfully
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

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()