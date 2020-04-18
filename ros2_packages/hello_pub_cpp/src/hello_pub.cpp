#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class HelloPub : public rclcpp::Node
{
public:
	HelloPub();
private:
	std::shared_ptr<std_msgs::msg::String> msg;
	void timerCB();
	rclcpp::TimerBase::SharedPtr timer;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
};
HelloPub::HelloPub() : Node("hello_pub")
{
	msg = std::make_shared<std_msgs::msg::String>();
	pub = this->create_publisher<std_msgs::msg::String>("/hello", 10);
	timer = this->create_wall_timer(100ms, std::bind(&HelloPub::timerCB, this));
}

void HelloPub::timerCB()
{
	msg->data = "Hello World!";
	RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());
	pub->publish(*msg);
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<HelloPub>());
	rclcpp::shutdown();
	
	return 0;
}
