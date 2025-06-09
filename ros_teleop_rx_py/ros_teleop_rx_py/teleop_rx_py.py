import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.teleop_callback,
            10)
        self.subscription  # unused variable warning 방지

    def teleop_callback(self, msg):
        self.get_logger().info(
            "Linear velocity: %.2f, Angular velocity: %.2f" % 
            (msg.linear.x, msg.angular.z)
        )

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
