import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class TurtleBot4Controller(Node):
    def __init__(self):
        # create node
        super().__init__('turtlebot4_controller')

        # link up publishers and/or subscribers
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def joy_callback(self, joy_msg):
        # create message and add values
        twist_msg = Twist()
        linear_speed = joy_msg.axes[1] * 0.5  # Left stick Y-axis for linear speed
        angular_speed = joy_msg.axes[0] * 0.5  # Left stick X-axis for angular speed

        # store values into msg
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_speed

        # publish and log msg
        self.publisher.publish(twist_msg)
        self.get_logger().info('Publishing cmd_vel: linear=%f, angular=%f' % (linear_speed, angular_speed))

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
