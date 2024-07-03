#include "rclcpp/rclcpp.hpp"
#include "ros_interfaces/srv/sleep_srv.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc != 3)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: ros2 run ros_task client type duration");
        return 1;
    }

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("client");
    rclcpp::Client<ros_interfaces::srv::SleepSrv>::SharedPtr client =
        node->create_client<ros_interfaces::srv::SleepSrv>("sleep_srv");

    auto request = std::make_shared<ros_interfaces::srv::SleepSrv::Request>();
    request->type = atoi(argv[1]);
    request->duration = atof(argv[2]);

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                         "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto res = result.get();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                           "Success: " << res->success
                                       << ", Msg: " << res->message);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service sleep_srv");
    }

    rclcpp::shutdown();
    return 0;
}