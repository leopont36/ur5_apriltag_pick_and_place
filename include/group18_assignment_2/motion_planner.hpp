#ifndef MOTION_PLANNER_HPP_
#define MOTION_PLANNER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "group18_assignment_2/action/move_to_pose.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <memory>
#include <thread>
#include <chrono>
#include <cmath>

class MotionPlanner : public rclcpp::Node
{
public:
    MotionPlanner();
    void initializeMoveIt();

private:
    using MoveToPose = group18_assignment_2::action::MoveToPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<MoveToPose>;

    // Action server callbacks
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveToPose::Goal> goal);
    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle);
    void execute(const std::shared_ptr<GoalHandle> goal_handle);
    void SetSafetyPose();
    void MoveToZeroPose();

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan plan_;
    rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;

    const std::vector<double> ZERO_POSE = {0.0, -M_PI_2, 0.0, -M_PI_2, 0.0, 0.0};
    const std::vector<double> SAFETY_POSE_POS = {-M_PI / 4.0, -2.0 * M_PI / 3.0, 7.0 * M_PI / 18.0, 0.0, 0.0, 0.0};
    const std::vector<double> SAFETY_POSE_NEG = {29.0 * M_PI / 36.0, -M_PI / 3.0, -7.0 * M_PI / 18.0, 0.0, 0.0, 0.0};

    double normalize_rad(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

#endif // MOTION_PLANNER_HPP_