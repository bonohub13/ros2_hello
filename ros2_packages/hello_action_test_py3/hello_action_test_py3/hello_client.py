# hello_client
from action_msgs.msg import GoalStatus
from hello_msgs.action import Hello

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# The Client tells the Server what the Client wants while the Server handles the process
class HelloClient(Node):
    def __init__(self):
        super().__init__('hello_action_test_client')

        self.hello_client = ActionClient(self, Hello, 'hello_action')

    def goal_responseCB(self, future): # dunno what the future handles...
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected: {0.accepted}'.format(goal_handle))
            return

        self.get_logger().info('Goal accepted: {0.accepted}'.format(goal_handle))

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_resultCB)

    def feedbackCB(self, feedback):
        # Gets feedback data by "goal_handle.publish_feedback" function from server
        self.get_logger().info('Received feedback:\n{0.status}'.format(feedback.feedback))

    def get_resultCB(self, future):
        result = future.result().result
        status = future.result().status # probably "goal_handle.succeed()" from server?

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!\n\t Result: {0.status}'.format(result))
        else:
            self.get_logger().info('Goal failed with status: {}'.format(status))

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self.hello_client.wait_for_server()

        goal_msg = Hello.Goal()
        goal_msg.hello_goal = 'Hello World!'

        self.get_logger().info('Sending goal request...')
        self.send_goal_future = self.hello_client.send_goal_async(goal_msg,
                feedback_callback=self.feedbackCB)
        self.send_goal_future.add_done_callback(self.goal_responseCB)

    def destroy(self):
        self.hello_client.destroy()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)

    hello_client = HelloClient()
    hello_client.send_goal()
    try:
        rclpy.spin(hello_client)
    except KeyboardInterrupt:
        pass
    hello_client.destroy()
    rclpy.shutdown()
