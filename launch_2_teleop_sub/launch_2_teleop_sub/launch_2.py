import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist  # Assuming teleop message is Twist

class MinimalPublisherSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_publisher_subscriber')
        self.publisher_ = self.create_publisher(String, 'topic_name', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Teleop subscriber setup
        self.teleop_subscription = self.create_subscription(
            Twist,  # Change this if your teleop message type is different
            'cmd_vel',  # Replace with the actual teleop topic name
            self.teleop_callback,
            100)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS2!'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def teleop_callback(self, msg):
        # You can access the received teleop data here (e.g., linear/angular velocities)
        self.get_logger().info(f'Received teleop message: linear_x: {msg.linear.x}')

        # If desired, you can use the received teleop data for further processing

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher_subscriber = MinimalPublisherSubscriber()
    rclpy.spin(minimal_publisher_subscriber)

    # Destroy the nodes explicitly
    minimal_publisher_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()