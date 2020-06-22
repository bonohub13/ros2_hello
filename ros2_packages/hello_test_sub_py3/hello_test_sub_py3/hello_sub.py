# hello_sub_py3
# for testing how spinning nodes in different threads work [Failure]
# from threading import Thread
import rclpy
from rclpy.node import Node
# for testing how to spin nodes in the background
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

# For checking the behavior of how the subscriber works!
# If you're going to add anything to this code make sure to add a comment to each line!
# Specification for comments:
# 1. The goal for why the line was added for the first place
# 2. What it does
class HelloSub(Node):
    # The expected message to recieve
    msg_expected = 'Hello World!'
    def __init__(self):
        super().__init__('hello_sub')
        self.hello_msg = String()
        self.hello_sub = self.create_subscription(String, 'hello_test', self.helloCB, 10)

        self.hello_sub

    def helloCB(self, msg):
        self.hello_msg.data = msg.data
        if msg.data == self.msg_expected:
            self.get_logger().info('Recieved: {0.data}'.format(msg))
        else:
            self.get_logger().error('Did not recieve the expected data!')

def original_spin(node):
    while 1:
        node.hello_sub
    # this function, method, does not work!
    # Solution: need to use rclpy.spin() in another thread

def main():
    rclpy.init()
    hello_sub = HelloSub()
    try:
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(hello_sub)
        executor.spin()
        print('Added thread!')
        # original_spin(hello_sub) # does not work!
        # 2 lines below are for testing out multithreaded workload in ROS2 [Failed attempt]
        # for spinning nodes in the background
        # spin_thread1 = Thread(target=rclpy.spin, args=(hello_sub, ))
        # spin_thread1.start()
    except KeyboardInterrupt:
        pass
    executor.shutdown()
    hello_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
