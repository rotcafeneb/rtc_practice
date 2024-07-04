#include "ros_action/action_server.hpp"

#include <functional>
#include <memory>
#include <thread>

ActionServer::ActionServer() : Node("action_server")
{
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<MoveAction>(
        this, "move_action", std::bind(&ActionServer::handle_goal, this, _1, _2),
        std::bind(&ActionServer::handle_cancel, this, _1),
        std::bind(&ActionServer::handle_accepted, this, _1));

    declare_parameters();
    rate_ = this->get_parameter("rate").as_double();

    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto cb = [this](const rclcpp::Parameter &p) {
        RCLCPP_INFO(this->get_logger(), "Set new rate: %f", p.as_double());
        this->rate_ = p.as_double();
    };
    cb_handle_ = param_subscriber_->add_parameter_callback("rate", cb);
}

void ActionServer::declare_parameters()
{
    rcl_interfaces::msg::ParameterDescriptor rateDesc;
    rateDesc.name = "rate";
    rateDesc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
    rateDesc.description = "Execution frequency";

    this->declare_parameter("rate", 1.0, rateDesc);
}

rclcpp_action::GoalResponse ActionServer::handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                      std::shared_ptr<const MoveAction::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with target position %d",
                goal->target_position);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleMoveAction> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionServer::handle_accepted(const std::shared_ptr<GoalHandleMoveAction> goal_handle)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ActionServer::execute, this, _1), goal_handle}.detach();
}

void ActionServer::execute(const std::shared_ptr<GoalHandleMoveAction> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(rate_);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveAction::Feedback>();
    auto &action_pos = feedback->position;
    auto result = std::make_shared<MoveAction::Result>();

    double speed = (goal->target_position - position_) / goal->duration;

    for (int idx = ceil(goal->duration * rate_); idx && rclcpp::ok(); --idx)
    {
        // Check if there is a cancel request
        if (goal_handle->is_canceling())
        {
            result->header.stamp = this->get_clock()->now();
            result->header.frame_id = "robot";
            result->message = "Action cancelled";
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }
        // Update
        position_ += speed / rate_;
        action_pos = position_;
        // Publish feedback
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback");

        loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok())
    {
        result->header.stamp = this->get_clock()->now();
        result->header.frame_id = "robot";
        result->message = "Success";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionServer>());
    rclcpp::shutdown();
    return 0;
}
