import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class HelloPub(Node):
    def __init__(self):
        super().__init__('hello_pub')
        topicName = '/hello'
        self.pub = self.create_publisher(String, topicName, 10)
        self.hello_msg = String()

        # timer
        timer_rate = 10/1000
        self.rate = self.create_timer(timer_rate, self.timerCB)

    def timerCB(self):
        self.hello_msg.data = 'Hello World!'
        self.pub.publish(self.hello_msg)
        self.get_logger().info(self.hello_msg)

def main(args=None):
    rclpy.init(args=args)
    hello_pub = HelloPub()
    try:
        rclpy.spin(hello_pub)
    except KeyboardInterrupt:
        print('killing process...')
    hello_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
