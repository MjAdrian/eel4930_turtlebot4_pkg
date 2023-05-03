#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.srv import GetCostmap
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped
import tf_transformations
from ament_index_python.packages import get_package_share_directory

import numpy as np
import matplotlib.pyplot as plt
import random
from sklearn.cluster import DBSCAN
from bresenham import bresenham
import time

from libs_autoro_navigation.camera_pose import CameraPose

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

        self.nav_ = BasicNavigator()

        # All maps are stored as a 2D Numpy array
        self.map_ = None                # Laser scan map only, no inflation, is_obstacle ? 100 : 0
        self.costmap_ = None            # Nav2 costmap with occupation probability 0-255
        self.binary_costmap_ = None     # Binary costmap with inflation (inflated_cell_value = 255)
        self.search_map_ = None         # Search area = map_ - (outside area + inside obstacles)
        self.traversable_map_ = None    # traversable_map_ = search_map_ - binary_costmap_

        self.width_ = 0                 # Width of maps
        self.height_ = 0                # Height of maps
        self.map_resolution_ = 0        # Resolution of map (m/cell)
        self.occupied_threshold_ = 0.50 # A cell is occupied if cost >= occupied_threshold
        self.coverage_threshold_ = 0.10 # Amount of the map the waypoint generator should attempt to cover
        self.camera_fov_ = 130          # Camera field of view in degrees
        self.camera_radius_ = 80        # Max camera visible distance in cells

        self.search_waypoints = []      # List of waypoints robot needs to traverse to see all points on map

        # Client to request global map
        self.get_map_client_ = self.create_client(GetMap, '/map_server/map')
        while not self.get_map_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service "/map_server/map" not available, waiting...')

        # Client to request global costmap
        self.get_costmap_client = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        while not self.get_costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service "/global_costmap/get_costmap" not available, waiting...')


    # Updates self.map_, self.costmap_, self.binary_costmap_, self.search_map_, self.traversable_map_
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
        self.map_resolution_ = map.info.resolution
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

        # '0' for unoccupied cell, '100' for cells with obstacle
        self.search_map_ = np.copy(self.map_)

        # First clustering, keep the cluster that represents the interior (search) space
        # Cell value in cluster map corresponds to cluster label (0 to n_clusters-1, -1 if not in cluster)
        labels, n_clusters, cluster_map = self.generate_clusters(self.search_map_)

        # Update search_map_ to only include cluster '1' (this is assumed to be the search space
        # if there is an unvisited border around the edge of the map)
        cluster_mask = cluster_map == 1.0
        self.search_map_[cluster_mask] = 0
        self.search_map_[~cluster_mask] = 100

        self.traversable_map_ = np.copy(self.search_map_)
        self.traversable_map_[occupied_mask] = 100

        
        self.visualize_map(self.map_)
        self.visualize_map(self.costmap_)
        self.visualize_map(self.binary_costmap_)
        self.visualize_map(self.search_map_)
        self.visualize_map(self.traversable_map_)

        self.get_logger().info('All maps updated!')


    # TODO: Generate waypoints for search path based on maps
    def generate_waypoints(self):
        # List of waypoints to traverse
        waypoint_list = []

        # Local map to keep track of not yet visible cells
        # '0' for unseen cell (all empty cells right now), '100' for cells with obstacle
        unseen_map = np.copy(self.search_map_)

        # Extract traversable points from traversable map
        traversable_points = []
        for y in range(len(self.traversable_map_)):
            for x in range(len(self.traversable_map_[0])):
                if self.traversable_map_[y, x] == 0:
                    traversable_points.append((x, y))

        # Get initial number of unseen points
        # num_unseen_points = 0
        # for row in unseen_map:
        #     for point in row:
        #         if point == 0:
        #             num_unseen_points = num_unseen_points + 1
        # initial_num_unseen_points = num_unseen_points

        # Define starting waypoint (not in waypoint list, but used to compute initial unseen_map)
        # Record points viewed from starting waypoint and update camera coverage
        # initial_pose = CameraPose()
        # initial_pose.x_ = 291
        # initial_pose.y_ = 357
        # initial_pose.theta_ = 220
        # new_visible_points = self.get_unique_visible_points(unseen_map, initial_pose, self.camera_fov_, self.camera_radius_, 0.5)
        # num_unseen_points = num_unseen_points - len(new_visible_points)
        # camera_coverage = (initial_num_unseen_points - num_unseen_points) / initial_num_unseen_points
        # self.shade_points(unseen_map, new_visible_points, 10)

        # Testing: Unseen map should be updated with initial waypoint points
        #self.visualize_map(unseen_map)

        # Randomly propose waypoints within binary_costmap_
        # Choose waypoints that can see the most unseen points
        # Recluster unseen points, repeat process on all clusters until coverage is acceptable
        theta_list = [0, 45, 90, 135, 180, 225, 270, 315]
        shade = 5
        y_lower_limit = 0
        y_upper_limit = 40
        while y_upper_limit+20 < self.height_:
            # Generate list of 5 random map points within traversable area and sufficiently far from each other
            test_pose_list = []
            for i in range(0,5):
                randx = random.randint(0, self.width_-1)
                randy = random.randint(y_lower_limit, y_upper_limit)
                counter = 1
                point_exists_in_range = True
                while self.traversable_map_[randy, randx] != 0:     # While point is not traversable
                    # If there are no feasible points in the range (more than 10 points have been tested)
                    if counter > 10:
                        point_exists_in_range = False
                        break
                    # Test another point and increment counter
                    randx = random.randint(0, self.width_-1)
                    randy = random.randint(y_lower_limit, y_upper_limit)
                    counter = counter + 1

                if point_exists_in_range:
                    # Add 8 poses (8 angles) for each point
                    for theta_idx in range(0,7):
                        camera_pose = CameraPose()
                        camera_pose.x_ = randx
                        camera_pose.y_ = randy
                        camera_pose.theta_ = theta_list[theta_idx]
                        test_pose_list.append(camera_pose)

            # Readjust limits
            y_lower_limit = y_lower_limit + 40
            y_upper_limit = y_upper_limit + 40

            if len(test_pose_list) > 0:
                # Iterate through 8 angles for every random point, choose the pose that
                # results in the greatest camera coverage (10*8) poses test total
                pose_scores = []    # Keeps track of the number of new visible points from each pose
                for camera_pose in test_pose_list:
                    print("Testing pose: (", camera_pose.x_, ",", camera_pose.y_, ",", camera_pose.theta_, ")")
                    new_visible_points = self.get_unique_visible_points(unseen_map, camera_pose, self.camera_fov_, self.camera_radius_, 0.25)
                    pose_scores.append(len(new_visible_points))

                # Choose the pose that produces the greatest coverage and update unseen_map
                # Update coverage percentage as well
                best_pose_idx = pose_scores.index(max(pose_scores))
                best_pose = test_pose_list[best_pose_idx]
                new_visible_points = self.get_unique_visible_points(unseen_map, best_pose, self.camera_fov_, self.camera_radius_, 0.25)
                self.shade_points(unseen_map, new_visible_points, shade)
                shade = shade + 5

                waypoint_list.append(best_pose)

                #self.visualize_map(unseen_map)

        self.visualize_map(unseen_map)
        return waypoint_list

    # Sends goal poses to navigation2 stack

    # Colors map based on list of coordinate points (x,y) provided
    def shade_points(self, map, points, shade):
        for point in points:
            map[point[1], point[0]] = shade

    # Returns a list of uniquely visible points from provided camera pose and currently visible points
    # Can increase degree_step=1/angular_resolution if there are holes in view cone
    # TODO: To speed up computation and improve resolution, project ray longer distance
    # and stop incrementing ray at camera radius (instead of making ray=camera radius)
    def get_unique_visible_points(self, unseen_map, camera_pose: CameraPose, fov, radius, resolution):

        visible_points = set()

        camera_x = camera_pose.x_
        camera_y = camera_pose.y_
        theta_ccw = camera_pose.theta_ - (fov/2)        # Counterclockwise FOV limit
        theta_cw = camera_pose.theta_ + (fov/2)         # Clockwise FOV limit

        # Make ray_length double the longest line that would be present on the map
        ray_length = 2*np.sqrt(self.width_**2 + self.height_**2)

        # Iterate through angles in FOV in resolution*degree increments
        for theta in np.linspace(theta_ccw, theta_cw, (int)(fov*resolution)):
            ray_endpoint_x = (int)(np.cos(np.radians(theta)) * ray_length) + camera_x
            ray_endpoint_y = (int)(np.sin(np.radians(theta)) * ray_length) + camera_y

            ray = list(bresenham(camera_x, camera_y, ray_endpoint_x, ray_endpoint_y))

            # Iterate through each point in ray until obstacle is encountered
            n = 0
            n_ray_x, n_ray_y = ray[n]

            # Iterate through points on ray until running into an obstacle
            while n_ray_y < self.height_ and n_ray_y >= 0 \
                    and n_ray_x < self.width_ and n_ray_x >= 0 \
                    and self.map_[n_ray_y, n_ray_x] == 0 \
                    and self.distance(n_ray_x, n_ray_y, camera_x, camera_y) < radius:
                # If point in not already in local visible_points and is in global unseen map,
                # add it to the visible set
                if unseen_map[n_ray_y, n_ray_x] == 0:
                    visible_points.add(ray[n])
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
        return labels, n_clusters, cluster_map

    # Computes 2D distance between 2 points
    def distance(self, x0, y0, x1, y1):
        return np.sqrt((x1-x0)**2 + (y1-y0)**2)

    # Displays provided map (numpy array) with matplotlib
    def visualize_map(self, map):
        fig, ax = plt.subplots()
        img = ax.imshow(map, cmap='rainbow')
        plt.show()

    def camera2mil_pose(self, camera_pose: CameraPose):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, camera_pose.theta_)
        nav_pose = PoseStamped()
        nav_pose.header.frame_id = 'map'
        nav_pose.pose.position.x = (camera_pose.x_*self.map_resolution_) - 8.67#(float)(camera_pose.x_ - 276) * self.map_resolution_#(5.86/self.width_)
        nav_pose.pose.position.y = (camera_pose.y_*self.map_resolution_) - 10.5#(float)(camera_pose.y_ - 359) * self.map_resolution_#(6.28/self.width_)
        nav_pose.pose.position.z = 0.0
        nav_pose.pose.orientation.x = q_x
        nav_pose.pose.orientation.y = q_y
        nav_pose.pose.orientation.z = q_z
        nav_pose.pose.orientation.w = q_w
        return nav_pose

    # Send initial pose to navigation stack
    # Blocks execution until pose set successfully
    def set_initial_pose(self):
        # Set initial pose
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 215.0)
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.nav_.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.x = q_x
        initial_pose.pose.orientation.y = q_y
        initial_pose.pose.orientation.z = q_z
        initial_pose.pose.orientation.w = q_w
        self.nav_.setInitialPose(initial_pose)

        # Wait for Nav2
        self.nav_.waitUntilNav2Active()

    def go_through_waypoints(self, waypoint_list):
        nav_waypoints = []
        for waypoint in waypoint_list:
            nav_waypoints.append(self.camera2mil_pose(waypoint))
            print(nav_waypoints[-1])
            #print("Nav Waypoint: (", nav_waypoints[-1].pose.position.x, nav_waypoints[-1].pose.position.y, ")")
        self.nav_.goThroughPoses(nav_waypoints)

    def test_waypoint(self):
        camera_pose = CameraPose()
        camera_pose.x_ = 252
        camera_pose.y_ = 120
        camera_pose.theta_ = 0
        goal_pose = self.camera2mil_pose(camera_pose)
        self.nav_.goToPose(goal_pose)
        

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()

    #node.set_initial_pose()
    node.update_maps()
    #node.test_waypoint()
    waypoint_list = node.generate_waypoints()
    node.go_through_waypoints(waypoint_list)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
