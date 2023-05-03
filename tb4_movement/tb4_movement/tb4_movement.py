import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from math import atan2

class movement_controller(Node):
    def __init__(self):
        '''
        __init__
            Constructor for when object is initialized. It creates a ROS2 node that subscribes to current_pose_goal
            and Publishes to cmd_vel. It also creates two method variables to store previous pose info to calculate
            velocity.
        '''
        # create node
        super().__init__('TB4_movement_controller')

        # link up publishers and/or subscribers
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber = self.create_subscription(PoseStamped, '/target_pose', self.pose_callback, 10)

        # method variables
        self.previous_pose = PoseStamped()
        self.previous_theta = 0

    def pose_callback(self, msg: PoseStamped):
        '''
        pose_callback
            Callback function when node recieves from current_pose_goal topic. It gets current pose and converts it to
            get the desired velocity for the TB4.

            Parameters:
                : msg: PoseStamped that is received from topic        
        '''
        # create message and add values

        # because the TB4 only rotates around the Z axis, this makes finding the cartesian angle easier to find
        # q = [cos(t/2), 0, 0, sin(t/2)]
        current_theta = 2* atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        
        # calculate velocities
        # assuming that this node recieves msg every 0.1 seconds
        vel_x = (msg.pose.position.x - self.previous_pose.pose.position.x)/0.1
        ang_vel_z = (current_theta - self.previous_theta)/0.1

        # create twist message
        twist_msg = Twist()

        # store values into twist msg
        # twist_msg.header = msg.header
        twist_msg.linear.x = vel_x
        twist_msg.angular.z = ang_vel_z

        # store pose for later calculation
        self.previous_pose = msg
        self.previous_theta = current_theta

        # publish and log msg
        if vel_x >= 0: 
            print(twist_msg)
            self.publisher.publish(twist_msg)
        # self.get_logger().info('Publishing cmd_vel: linear=%f, angular=%f' % (linear_speed, angular_speed))

def main(args=None):
    # Run node
    rclpy.init(args=args)
    tb4_controller = movement_controller()

    # spin until ctrl+c is pressed
    try:
        rclpy.spin(tb4_controller)
    except KeyboardInterrupt:
        print('TurtleBot4 controller interrupted by user. Shutting down...')
    finally:
        tb4_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
