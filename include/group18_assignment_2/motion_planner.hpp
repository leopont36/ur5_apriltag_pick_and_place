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

/**
 * @brief Node responsible for the kinematic planning and execution of the arm movements.
 * This class implements a ROS 2 Action Server that interfaces with the MoveIt framework
 * to translate Cartesian goals into joint trajectories. It incorporates a robust 
 * recovery strategy to handle planning failures in the constrained environment 
 * by utilizing intermediate safety configurations.
 */
class MotionPlanner : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the MotionPlanner node.
     * Initializes the ROS 2 node and starts the action server to listen for 
     * movement requests from the coordinator.
     */
    MotionPlanner();

    /**
     * @brief Configures the MoveIt Planning Interface.
     * Sets the planning group to "ir_arm" and defines the velocity and acceleration 
     * scaling factors. This initialization is separated from the constructor to 
     * ensure the shared pointer is fully initialized before use.
     */
    void initializeMoveIt();

private:
    using MoveToPose = group18_assignment_2::action::MoveToPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<MoveToPose>;

    // Action server callbacks for the lifecycle management of the goal
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveToPose::Goal> goal);
    void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle);

    /**
     * @brief Core logic for trajectory execution.
     * This method implements a hierarchical planning strategy. It first attempts 
     * to reach the target with strict orientation constraints to ensure object stability.
     * If the planning fails, it triggers a recovery routine involving the safety pose
     * and retries the movement with relaxed constraints.
     * @param goal_handle The handle to interact with the action client.
     */
    void execute(const std::shared_ptr<GoalHandle> goal_handle);

    /**
     * @brief Moves the arm to a predefined intermediate configuration.
     * This method is invoked during the recovery phase to move the robot away 
     * from singularities or collision-prone configurations near the table, 
     * resetting the kinematic state before a retry.
     */
    void SetSafetyPose();

    /**
     * @brief Moves the arm to the home configuration.
     * Executed at the end of the task to bring the robot back to its initial state.
     */
    void MoveToZeroPose();

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan plan_;
    rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;

    // Constant definitions for the home and safety configurations
    const std::vector<double> ZERO_POSE = {0.0, -M_PI_2, 0.0, -M_PI_2, 0.0, 0.0};
    const std::vector<double> SAFETY_POSE_POS = {-M_PI / 4.0, -2.0 * M_PI / 3.0, 7.0 * M_PI / 18.0, 0.0, 0.0, 0.0};
    const std::vector<double> SAFETY_POSE_NEG = {29.0 * M_PI / 36.0, -M_PI / 3.0, -7.0 * M_PI / 18.0, 0.0, 0.0, 0.0};

    /**
     * @brief Normalizes an angle to the range [-PI, PI].
     * @param angle The angle in radians to normalize.
     * @return The normalized angle.
     */
    double normalize_rad(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

#endif // MOTION_PLANNER_HPP_