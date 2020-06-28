// Client side (using Action)
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "hello_msgs/action/hello.hpp"

class HelloClient : public rclcpp::Node
{
public: // alias, constructors, and destructors
    using Hello = hello_msgs::action::Hello;
    using GoalHandleHello = rclcpp_action::ClientGoalHandle<Hello>;

    explicit HelloClient(const rclcpp::NodeOptions &options);
public: // functions
    bool is_goal_done() const;
    void send_goal();
private: // variables and instances
    rclcpp_action::Client<Hello>::SharedPtr action_client;
    rclcpp::TimerBase::SharedPtr timer;
    bool goal_done;
private: // functions
    void goal_responseCB(std::shared_future<GoalHandleHello::SharedPtr> future);
    void feedbackCB(
        GoalHandleHello::SharedPtr,
        const std::shared_ptr<const Hello::Feedback> feedback
    );
    void resultCB(const GoalHandleHello::WrappedResult &result);
};
HelloClient::HelloClient(const rclcpp::NodeOptions &options=rclcpp::NodeOptions())
    : Node("hello_action_test_client", options), goal_done(false)
{
    this->action_client = rclcpp_action::create_client<Hello>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "hello_action"
    );
    this->timer = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&HelloClient::send_goal, this)
    );
}
bool HelloClient::is_goal_done() const
{
    return this->goal_done;
}
void HelloClient::send_goal()
{
    using namespace std::placeholders;

    this->timer->cancel();
    this->goal_done = false;

    if (!this->action_client)
        RCLCPP_ERROR(this->get_logger(), "Action client not initialized");

    if (!this->action_client->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        this->goal_done = true;
        return;
    }

    auto goal_msg = Hello::Goal();
    goal_msg.hello_goal = "Hello World!";

    RCLCPP_INFO(this->get_logger(), "Sending goal to server");
    
    auto send_goal_options = rclcpp_action::Client<Hello>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&HelloClient::goal_responseCB, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&HelloClient::feedbackCB, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&HelloClient::resultCB, this, _1);
    auto goal_handle_future =
        this->action_client->async_send_goal(goal_msg, send_goal_options);
}
void HelloClient::goal_responseCB(std::shared_future<GoalHandleHello::SharedPtr> future)
{
    auto goal_handle = future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal rejected from server");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for results");
    }
}
void HelloClient::feedbackCB(
    GoalHandleHello::SharedPtr,
    const std::shared_ptr<const Hello::Feedback> feedback
)
{
    RCLCPP_INFO(this->get_logger(), "Current status: %s", feedback->status.c_str());
}
void HelloClient::resultCB(const GoalHandleHello::WrappedResult &result)
{
    this->goal_done = true;
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "GOAL ABORTED!");
        return;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "GOAL CANCELED!");
        return;
    default:
        RCLCPP_ERROR(this->get_logger(), "UNKNOWN RESULT CODE!");
        break;
    }

    RCLCPP_INFO(this->get_logger(), "Result received\n\tresult: %s", result.result->status.c_str());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto hello_action_client = std::make_shared<HelloClient>();

    while (!hello_action_client->is_goal_done())
    {
        rclcpp::spin_some(hello_action_client);
    }
    
    rclcpp::shutdown();
}