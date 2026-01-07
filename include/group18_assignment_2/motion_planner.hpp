#ifndef MOTION_PLANNER_HPP_
#define MOTION_PLANNER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/node_options.hpp"

#include "group18_assignment_2/action/move_to_pose.hpp"

#include <memory>
#include <thread>
#include <chrono>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

class MotionPlanner : public rclcpp::Node
{
public:
    MotionPlanner();
    void initializeMoveIt();
private:
    using MoveToPose = group18_assignment_2::action::MoveToPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<MoveToPose>;

    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveToPose::Goal> goal);
    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle);
    void execute(const std::shared_ptr<GoalHandle> goal_handle);


    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan plan_;
    rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
};  

#endif // MOTION_PLANNER_HPP_