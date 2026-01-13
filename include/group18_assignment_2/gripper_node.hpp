#ifndef GRIPPER_NODE_HPP_
#define GRIPPER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include "group18_assignment_2/action/gripper.hpp"

#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

/**
 * @brief Node responsible for the low-level control of the end-effector.
 * * This class realizes a ROS 2 Action Server that interfaces with the MoveIt framework
 * to control the pneumatic gripper. It abstracts the complexity of the kinematic 
 * chain planning, exposing a high-level interface to the supervisor node.
 * The architecture allows for both synchronous operations (open/close) and 
 * asynchronous operations (grasping with stall detection).
 */
class GripperNode : public rclcpp::Node
{
public:
    using GripperAction = group18_assignment_2::action::Gripper;
    using GoalHandleGripper = rclcpp_action::ServerGoalHandle<GripperAction>;

    /**
     * @brief Constructor for the GripperNode.
     * Initializes the ROS 2 infrastructure and starts the action server, waiting
     * for incoming requests from the coordination layer.
     */
    GripperNode();

private:
    rclcpp_action::Server<GripperAction>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

    /**
     * @brief Lazy initialization of the MoveIt components.
     * * Configures the planning group "ir_gripper" and sets the necessary kinematic
     * parameters. This method ensures that the heavy MoveIt initialization occurs 
     * only when the first goal is received, optimizing the startup time.
     */
    void init_moveit();

    /**
     * @brief Validates the incoming goal request.
     * @return GoalResponse::ACCEPT_AND_EXECUTE if the request is valid.
     */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GripperAction::Goal> goal);

    /**
     * @brief Handles the cancellation requests from the client.
     * Stops the current trajectory execution immediately to ensure safety.
     */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleGripper> goal_handle);

    /**
     * @brief Callback executed when a goal is accepted.
     * Spawns a dedicated thread to execute the movement logic without blocking
     * the ROS 2 executor.
     */
    void handle_accepted(
        const std::shared_ptr<GoalHandleGripper> goal_handle);

    /**
     * @brief Core execution logic for the gripper control.
     * * Distinguishes between standard movements (open/close), which are executed
     * synchronously, and the grasping maneuver, which utilizes an asynchronous
     * strategy to handle physical collisions with the object.
     * @param goal_handle The handle to interact with the action client.
     */
    void execute(const std::shared_ptr<GoalHandleGripper> goal_handle);
};

#endif // GRIPPER_NODE_HPP_