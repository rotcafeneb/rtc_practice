#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"

class SimplePublisher : public rclcpp::Node
{
  public:
    SimplePublisher();

  private:
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr publisher_;
    size_t count_;
};
