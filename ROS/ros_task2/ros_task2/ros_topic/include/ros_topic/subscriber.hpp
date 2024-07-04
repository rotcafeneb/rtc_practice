#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"

class SimpleSubscriber : public rclcpp::Node
{
  public:
    SimpleSubscriber();

  private:
    void topic_callback(const builtin_interfaces::msg::Time::SharedPtr msg) const;

    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr subscription_;
};
