// Server side (using Action)
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp" // may not be required if implemented into rclcpp
#include "hello_msgs/action/hello.hpp"

class HelloServer : public rclcpp::Node
{
public:
	/* call using HelloServer::hoge when out of scope */
    using Hello = hello_msgs::action::Hello;
    using GoalHandleHello = rclcpp_action::ServerGoalHandle<Hello>;

    explicit HelloServer(const rclcpp::NodeOptions &options);
private: //variables and instances
    rclcpp_action::Server<Hello>::SharedPtr action_server;
private: //functions
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Hello::Goal> goal
    );
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleHello> goal_handle);
    void execute(const std::shared_ptr<GoalHandleHello> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleHello> goal_handle);
};
HelloServer::HelloServer(const rclcpp::NodeOptions &options=rclcpp::NodeOptions())
    : Node("hello_action_test_server", options)
{
    using namespace std::placeholders;

    this->action_server = rclcpp_action::create_server<Hello>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "hello_action",
        std::bind(&HelloServer::handle_goal, this, _1, _2),
        std::bind(&HelloServer::handle_cancel, this, _1),
        std::bind(&HelloServer::handle_accepted, this, _1)
    );
}
rclcpp_action::GoalResponse HelloServer::handle_goal(
	const rclcpp_action::GoalUUID &uuid,
	std::shared_ptr<const HelloServer::Hello::Goal> goal
)
{
	RCLCPP_INFO(this->get_logger(), "Received goal request:\n\thello_goal: %s", goal->hello_goal.c_str());
	(void)uuid; //why???
	if (goal->hello_goal != "Hello World!") //reject goal that is not "Hello World!"
		return rclcpp_action::GoalResponse::REJECT;
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse HelloServer::handle_cancel(const std::shared_ptr<HelloServer::GoalHandleHello> goal_handle)
{
	RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
	(void)goal_handle; //why???
	return rclcpp_action::CancelResponse::ACCEPT;
}
void HelloServer::execute(const std::shared_ptr<HelloServer::GoalHandleHello> goal_handle)
{
	RCLCPP_INFO(this->get_logger(), "Executing goal.");
	rclcpp::Rate loop_rate(1);
	const auto goal = goal_handle->get_goal(); // instance to store goal
	auto feedback = std::make_shared<Hello::Feedback>(); // making instance for feedback
	/* initializing feedback */
	std::string &status = feedback->status;
	status = "";

	auto result = std::make_shared<Hello::Result>(); // making instance for result

	bool complete = false;

	while (!complete)
	{
		if (goal_handle->is_canceling())
		{
			result->status = status;
			goal_handle->canceled(result);
			RCLCPP_INFO(this->get_logger(), "Goal Canceled.");
			return ;
		}
		status += goal->hello_goal[status.length()]; // update Hello status

		/* Publish Feedback */
		RCLCPP_INFO(this->get_logger(), "Publishing feedback: %s", status.c_str());
		goal_handle->publish_feedback(feedback);

		complete = status == goal->hello_goal;

		loop_rate.sleep();
	}

	if (rclcpp::ok())
	{
		result->status = status;
		goal_handle->succeed(result);
		RCLCPP_INFO(this->get_logger(), "Goal Succeeded!");
	}
}
void HelloServer::handle_accepted(const std::shared_ptr<HelloServer::GoalHandleHello> goal_handle)
{
	using namespace std::placeholders;
	
	std::thread{std::bind(&HelloServer::execute, this, _1), goal_handle}.detach();
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	
	auto hello_action_server = std::make_shared<HelloServer>();

	rclcpp::spin(hello_action_server);
	rclcpp::shutdown();
}
