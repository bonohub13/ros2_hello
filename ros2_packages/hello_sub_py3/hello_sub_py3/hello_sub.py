import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class HelloSub(Node):
    def __init__(self):
        super().__init__('hello_sub')
        topicName = '/hello'
        self.hello_msg = String()
        self.sub = self.create_subscription(String, 
                                            topicName, 
                                            self.helloCB, 
                                            10)
        self.sub

    def helloCB(self, msg):
        self.hello_msg = msg
        self.get_logger().info(msg)

def main(args=None):
    rclpy.init(args=None)
    hello_sub = HelloSub()
    rclpy.spin(hello_sub)
    hello_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
