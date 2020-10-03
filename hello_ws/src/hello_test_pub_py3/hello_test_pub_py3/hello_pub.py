# hello_pub_py3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# For checking the behavior of how the publisher works!
# If you're going to add anything to this code make sure to add a comment to each line!
# Specification for comments:
# 1. The goal for why the line was added for the first place
# 2. What it does
class HelloPub(Node):
    msg_send = 'Hello World!'
    def __init__(self):
        super().__init__('hello_pub')
        self.hello_msg = String()
        self.hello_msg.data = self.msg_send
        self.hello_pub = self.create_publisher(String, 'hello_test', 10)
        self.timer = self.create_timer(1e-1, self.timerCB)

    def timerCB(self):
        self.hello_pub.publish(self.hello_msg)
        self.get_logger().info('Sent: {0.data}'.format(self.hello_msg))

def main():
    rclpy.init()
    hello_pub = HelloPub()
    try:
        rclpy.spin(hello_pub)
    except KeyboardInterrupt:
        hello_pub.destroy_node()
        rclpy.shutdown()
        exit()
    hello_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
