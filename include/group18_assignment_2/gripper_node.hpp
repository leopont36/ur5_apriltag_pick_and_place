#ifndef GRIPPER_NODE_HPP_
#define GRIPPER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include "group18_assignment_2/action/gripper.hpp"

class GripperNode : public rclcpp::Node
{
public:
    using GripperAction = group18_assignment_2::action::Gripper;
    using GoalHandleGripper = rclcpp_action::ServerGoalHandle<GripperAction>;

    GripperNode();

private:
    rclcpp_action::Server<GripperAction>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

    void init_moveit();

    // Action Callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GripperAction::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleGripper> goal_handle);

    void handle_accepted(
        const std::shared_ptr<GoalHandleGripper> goal_handle);

    // The actual movement logic
    void execute(const std::shared_ptr<GoalHandleGripper> goal_handle);
};

#endif // GRIPPER_NODE_HPP_