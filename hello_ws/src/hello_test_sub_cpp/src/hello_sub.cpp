/* hello_sub */
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class HelloSub : public rclcpp::Node
{
public:
    using String = std_msgs::msg::String;

    explicit HelloSub(const rclcpp::NodeOptions &options);
private:
    String msg;
    rclcpp::Subscription<String>::SharedPtr sub;
private:
    void helloCB(const String::SharedPtr msg);
};
HelloSub::HelloSub(const rclcpp::NodeOptions &options=rclcpp::NodeOptions())
    : Node("hello_sub", options)
{
    using namespace std::placeholders;

    this->sub = this->create_subscription<String>(
        "/hello",
        10,
        std::bind(&HelloSub::helloCB, this, _1));
}
void HelloSub::helloCB(const String::SharedPtr msg)
{
    this->msg.data = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto hello_sub = std::make_shared<HelloSub>();
    rclcpp::spin(hello_sub);

    rclcpp::shutdown();
}