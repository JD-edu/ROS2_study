
import rclpy
from rclpy.node import Node
from uga_msg.msg import MsgServo


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(MsgServo, 'topic_servo_jjy', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = MsgServo()
        msg.degree = self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.degree)
        self.i += 5
        if self.i > 90:
            self.i = 0


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher() # This is NODE object.

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()