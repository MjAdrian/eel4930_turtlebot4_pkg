import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleBot4Controller(Node):
    def __init__(self):
        # create node
        super().__init__('turtlebot4_controller')

        # link up publishers and/or subscribers
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # timer for publisher
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.move_callback)

    def move_callback(self):
        # create message and add values
        twist_msg = Twist()
        linear_speed = 0.5  # meters per second
        angular_speed = 0.0  # radians per second

        # store values into msg
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_speed

        # publish and log msg
        self.publisher.publish(twist_msg)
        # self.get_logger().info('Publishing cmd_vel: linear=%f, angular=%f' % (linear_speed, angular_speed))

def main(args=None):
    # Run node
    try:
        rclpy.init(args=args)
        tb4_controller = TurtleBot4Controller()
        rclpy.spin(tb4_controller)
    # if CTRL+C is pressed
    except KeyboardInterrupt:
        print('TurtleBot4 controller interrupted by user. Shutting down...')
    # kill node
    finally:
        if tb4_controller:
            tb4_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
