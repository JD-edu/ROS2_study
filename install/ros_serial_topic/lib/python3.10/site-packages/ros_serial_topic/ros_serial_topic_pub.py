import rclpy
from rclpy.node import Node 
from std_msgs.msg import String
import serial 

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic_serial', 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0
        self.ser = serial.Serial("/dev/ttyACM0", baudrate = 115200, parity=serial.PARITY_NONE, 
                               stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, 
                               timeout=0.3)
        self.read_serial()

    def read_serial(self):
        while True:
            msg = String()
            msg.data = self.ser.readline().decode()
            self.publisher_.publish(msg)
            self.get_logger().info(msg.data)
            

def main(args=None):
    print("start topic...")
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()