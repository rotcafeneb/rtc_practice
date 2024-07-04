#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros_interfaces/action/move.hpp"

class ActionServer : public rclcpp::Node
{
  public:
    using MoveAction = ros_interfaces::action::Move;
    using GoalHandleMoveAction = rclcpp_action::ServerGoalHandle<ros_interfaces::action::Move>;

    ActionServer();

    double rate_ = 1;

  private:
    rclcpp_action::Server<MoveAction>::SharedPtr action_server_;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
    double position_ = 0;

    void declare_parameters();

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const MoveAction::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveAction> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleMoveAction> goal_handle);

    void execute(const std::shared_ptr<GoalHandleMoveAction> goal_handle);
};
