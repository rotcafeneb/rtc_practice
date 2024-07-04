#include "rclcpp/rclcpp.hpp"
#include "ros_interfaces/srv/sleep_srv.hpp"

class SimpleService : public rclcpp::Node
{
  public:
    SimpleService();

  private:
    void handler(const std::shared_ptr<ros_interfaces::srv::SleepSrv::Request> request,
                 std::shared_ptr<ros_interfaces::srv::SleepSrv::Response> response);

    rclcpp::Service<ros_interfaces::srv::SleepSrv>::SharedPtr service_;
};
