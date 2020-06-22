# hello_executor_test_py3
import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from hello_test_pub_py3 import hello_pub
from hello_test_sub_py3 import hello_sub

# For checking the behavior of how executors works!
# If you're going to add anything to this code make sure to add a comment to each line!
# Specification for comments:
# 1. The goal for why the line was added for the first place
# 2. What it does
class HelloExecutor:
    def __init__(self, nodes: list=[]):
        self.nodes = nodes
        # self.executor = SingleThreadedExecutor() # commented out to test out MultiThreadedExecutor and see how it works
        self.executor = MultiThreadedExecutor(num_threads=2) # if num_threads is 1, it executes as single thread
        if len(nodes) is 0:
            self.executor.shutdown()
            rclpy.shutdown()
            raise ValueError('Not any node was given!')
        elif len(nodes) > 1:
            for node in nodes:
                self.executor.add_node(node)
        else:
            self.executor.add_node(node)

    def spin(self):
        try:
            self.executor.spin()
        finally:
            print('Killing executor and nodes')
            self.executor.shutdown()
            for node in self.nodes:
                node.destroy_node()

def main():
    rclpy.init()
    h_pub = hello_pub.HelloPub()
    h_sub = hello_sub.HelloSub()
    hello_exec = HelloExecutor(nodes=[h_pub, h_sub])
    try:
        hello_exec.spin()
    except KeyboardInterrupt:
        print('Process aborted!')

if __name__ == '__main__':
    main()
