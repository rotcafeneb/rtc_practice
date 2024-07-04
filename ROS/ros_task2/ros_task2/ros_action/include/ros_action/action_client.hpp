#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros_interfaces/action/move.hpp"

class ActionClient : public rclcpp::Node
{
  public:
    using MoveAction = ros_interfaces::action::Move;
    using GoalHandleMoveAction = rclcpp_action::ClientGoalHandle<MoveAction>;

    ActionClient();
    void send_goal(const int32_t target, const float duration);

  private:
    rclcpp_action::Client<MoveAction>::SharedPtr client_ptr_;

    void goal_response_callback(const GoalHandleMoveAction::SharedPtr &goal_handle);

    void feedback_callback(GoalHandleMoveAction::SharedPtr,
                           const std::shared_ptr<const MoveAction::Feedback> feedback);

    void result_callback(const GoalHandleMoveAction::WrappedResult &result);
};
