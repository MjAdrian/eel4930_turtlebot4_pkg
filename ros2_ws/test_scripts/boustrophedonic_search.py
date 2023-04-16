from math import cos, sin
import numpy as np
import rclpy
from geometry_msgs.msg import Point32, Polygon
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav2_simple_commander.line_iterator import LineIterator

class BoustrophedonicSearch(Node):
    def __init__(self):
        super().__init__('boustrophedonic_search')
        self.declare_parameter('map_resolution', 0.1)   # Resolution of costmap
        self.declare_parameter('path_resolution', 2)    # Cell resolution of path
        self.declare_parameter('robot_radius', 0.2)     # Radius of robot for costmap inflation
        self.declare_parameter('line_spacing', 0.4)     # Distance between scan lines
        self.declare_parameter('scan_direction', 'h')   # Horizontal or vertical scan lines
        self.my_var = 1

        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            'costmap',
            self.costmap_callback,
            1
        )

        self.path_pub = self.create_publisher(Path, 'path', 1)
    
    def costmap_callback(self, msg):
        costmap = PyCostmap2D(msg.info.wdith, msg.info.height, np.array(msg.data))
        inflated_costmap = costmap.inflate(self.robot_radius)

        # Don't know how often costmap is update, maybe don't regenerate path every time?
        #waypoints = self.generate_boustrophedonic_path(inflated_costmap)

        if len(waypoints) > 0:
            path = self.create_path_msg(waypoints, msg.header.frame_id)
            self.path_pub.publish(path)

    def costmap_callback(self, msg):
        costmap = Costmap2D(msg.info.width, msg.info.height, np.array(msg.data))
        inflated_costmap = costmap.inflate(self.robot_radius)

        waypoints = self.generate_boustrophedonic_path(inflated_costmap)

        if len(waypoints) > 0:
            path = self.create_path_msg(waypoints, msg.header.frame_id)
            self.path_pub.publish(path)
        else:
            self.get_logger().warn('No waypoints generated.')

    def generate_boustrophedonic_path(self, costmap):
        # Implement your boustrophedonic path generation algorithm here
        # based on the inflated costmap
        waypoints = []
        # Iterate through the costmap cells in a boustrophedonic pattern
        for y in range(costmap.height):     # Iterate through each vertical cell
            if y % 2 == 0:                  # For every other cell
                for x in range(costmap.width):      # Iterate through every horizontal cell in row
                    # Check if the cell is free and has enough distance from the previous waypoint
                    if costmap.is_free(x, y) and self.has_min_distance(x, y, waypoints):
                        waypoints.append((x, y))
                    else:
                        # Propose either splitting up or splitting down (prefer split up)
                        # 10 is arbritrary max obstacle "height". Maybe find better way of doing this
                        for y_offset in range(1, 10, self.path_resolution, )
                        

    def create_path_msg(self, waypoints, frame_id):
        # Create a Path ROS message from the generated waypoints
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id
        for point in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            path.poses.append(pose)
            return path

    def main(args=None):
        rclpy.init(args=args)
        node = BoustrophedonicPathGenerator()
        rclpy.spin(node)
        rclpy.shutdown()

    if __name__ == 'main':
        main()