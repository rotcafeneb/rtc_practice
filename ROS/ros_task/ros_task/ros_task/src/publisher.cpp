#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node
{
  public:
    SimplePublisher() : Node("publisher"), count_(0)
    {
        publisher_ =
            this->create_publisher<builtin_interfaces::msg::Time>("topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&SimplePublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        auto message = builtin_interfaces::msg::Time();
        message = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Publishing: '%d' s '%d' ns",
                    message.sec, message.nanosec);
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePublisher>());
    rclcpp::shutdown();
    return 0;
}