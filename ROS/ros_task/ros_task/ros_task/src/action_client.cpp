#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "ros_interfaces/action/move.hpp"


class ActionClient : public rclcpp::Node
{
  public:
    using MoveAction = ros_interfaces::action::Move;
    using GoalHandleMoveAction = rclcpp_action::ClientGoalHandle<MoveAction>;

    ActionClient()
        : Node("action_client")
    {
        this->client_ptr_ = rclcpp_action::create_client<MoveAction>(this, "move_action");
    }

    void send_goal(const int32_t target, const float duration)
    {
        using namespace std::placeholders;

        if (!this->client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = MoveAction::Goal();
        goal_msg.target_position = target;
        goal_msg.duration = duration;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<MoveAction>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&ActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&ActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&ActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

  private:
    rclcpp_action::Client<MoveAction>::SharedPtr client_ptr_;

    void goal_response_callback(const GoalHandleMoveAction::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(GoalHandleMoveAction::SharedPtr,
                           const std::shared_ptr<const MoveAction::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Current position: %d", feedback->position);
    }

    void result_callback(const GoalHandleMoveAction::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        
        RCLCPP_INFO_STREAM(this->get_logger(), "Result: " << result.result->message);
        rclcpp::shutdown();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    if (argc != 3)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: ros2 run ros_task client {target} {duration}");
        return 1;
    }

    auto node = std::make_shared<ActionClient>();

    node->send_goal(atoi(argv[1]), atof(argv[2]));
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}