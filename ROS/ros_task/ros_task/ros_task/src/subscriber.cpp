#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class SimpleSubscriber : public rclcpp::Node
{
  public:
    SimpleSubscriber() : Node("subscriber")
    {
        subscription_ = this->create_subscription<builtin_interfaces::msg::Time>(
            "topic", 10, std::bind(&SimpleSubscriber::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const builtin_interfaces::msg::Time::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Publishing: '%d' s '%d' ns", msg->sec, msg->nanosec);
    }
    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleSubscriber>());
    rclcpp::shutdown();
    return 0;
}