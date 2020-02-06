#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class HelloSub : public rclcpp::Node
{
public:
	HelloSub();
private:
	void topicCB(const std_msgs::msg::String::SharedPtr msg);
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
	std_msgs::msg::String hello_msg;
};
HelloSub::HelloSub() : Node("hello_sub")
{
	sub = this->create_subscription<std_msgs::msg::String>("/hello", 10, std::bind(&HelloSub::topicCB, this, _1));
}

void HelloSub::topicCB(const std_msgs::msg::String::SharedPtr msg)
{
	hello_msg.data = msg->data;
	RCLCPP_INFO(this->get_logger(), "subscribed: %s", hello_msg.data.c_str());
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<HelloSub>());
	rclcpp::shutdown();

	return 0;
}
