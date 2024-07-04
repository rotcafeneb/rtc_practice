#include "ros_service/service.hpp"

#include <chrono>
#include <memory>
#include <thread>

SimpleService::SimpleService() : Node("service")
{
    service_ = this->create_service<ros_interfaces::srv::SleepSrv>(
        "sleep_srv",
        std::bind(&SimpleService::handler, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Service init success");
}

void SimpleService::handler(const std::shared_ptr<ros_interfaces::srv::SleepSrv::Request> request,
                            std::shared_ptr<ros_interfaces::srv::SleepSrv::Response> response)
{
    switch (request->type)
    {
    case ros_interfaces::srv::SleepSrv::Request::NOW:
        response->success = 1;
        break;

    case ros_interfaces::srv::SleepSrv::Request::DEFERRED:
        std::this_thread::sleep_for(std::chrono::duration<float>(request->duration));
        response->success = 1;
        break;

    default:
        response->success = 0;
        response->message = "Wrong type";
        break;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleService>());
    rclcpp::shutdown();
    return 0;
}