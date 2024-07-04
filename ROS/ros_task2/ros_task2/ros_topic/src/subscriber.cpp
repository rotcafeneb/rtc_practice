#include "ros_topic/subscriber.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

SimpleSubscriber::SimpleSubscriber() : Node("subscriber")
{
    subscription_ = this->create_subscription<builtin_interfaces::msg::Time>(
        "topic", 10, std::bind(&SimpleSubscriber::topic_callback, this, std::placeholders::_1));
}

void SimpleSubscriber::topic_callback(const builtin_interfaces::msg::Time::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "Received: '%d' s '%d' ns", msg->sec, msg->nanosec);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleSubscriber>());
    rclcpp::shutdown();
    return 0;
}