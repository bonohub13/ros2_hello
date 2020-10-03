/* hello_pub */
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class HelloPub : public rclcpp::Node
{
public:
    using String = std_msgs::msg::String;

    explicit HelloPub(const rclcpp::NodeOptions &options);
private: // variables and instances
    String msg;
    rclcpp::Publisher<String>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer;
private: // functions
    void timerCB();
};
HelloPub::HelloPub(const rclcpp::NodeOptions &options=rclcpp::NodeOptions())
    : Node("hello_pub", options)
{
    using namespace std::chrono_literals;

    this->pub = this->create_publisher<String>("/hello", 10);
    this->timer = this->create_wall_timer(100ms, std::bind(&HelloPub::timerCB, this));
}
void HelloPub::timerCB()
{
    msg.data = "Hello World!";
    RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());
    pub->publish(msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto hello_pub = std::make_shared<HelloPub>();
    rclcpp::spin(hello_pub);

    rclcpp::shutdown();
}