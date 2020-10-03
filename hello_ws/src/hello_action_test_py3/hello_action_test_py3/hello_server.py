# hello_server
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor # may not be required
from rclpy.node import Node

from hello_msgs.action import Hello

# The Server handles the main process while the Client just orders the Server what it wants
class HelloServer(Node):
    def __init__(self):
        super().__init__('hello_action_test_server')

        self.hello_server = ActionServer(self, Hello, 'hello_action',
                execute_callback=self.executeCB,
                callback_group=ReentrantCallbackGroup(),
                goal_callback=self.goalCB,
                cancel_callback=self.cancelCB)

    def destroy(self): # cleanup
        self.hello_server.destroy()
        self.destroy_node()

    def goalCB(self, goal_request): # callback for goal
        ### Accepts or rejects a client request to begin an action. ###
        self.get_logger().info('Received goal request:\n{}'.format(goal_request))
        return GoalResponse.ACCEPT # returns to client that a request to begin an action has been accepted.

    def cancelCB(self, goal_handle):
        ### Accepts or rejects a client request to cancel an action. ###
        self.get_logger().info('Received cancel request:\n{}'.format(goal_handle))
        return CancelResponse.ACCEPT # returns to client that a request to cancel an action has been accepted.

    async def executeCB(self, goal_handle): # corutine for executing goal
        self.get_logger().info('Executing goal...')

        # Initial input for Hello
        feedback_msg = Hello.Feedback()
        feedback_msg.status = ''

        complete = False

        # Start execution of action
        while not complete:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Hello.Result()

            # update Hello status
            feedback_msg.status += goal_handle.request.hello_goal[len(feedback_msg.status)]
            
            # publish feedback
            self.get_logger().info('Publishing feedback: {0.status}'.format(feedback_msg))
            goal_handle.publish_feedback(feedback_msg)

            complete = feedback_msg.status == goal_handle.request.hello_goal

        goal_handle.succeed()

        # Populate result message
        result = Hello.Result()
        result.status = feedback_msg.status

        self.get_logger().info('Result: {0.status}'.format(result))

        return result

def main(args=None):
    rclpy.init(args=args)

    hello_server = HelloServer()

    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(hello_server, executor=executor)
    except KeyboardInterrupt:
        pass
    hello_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
