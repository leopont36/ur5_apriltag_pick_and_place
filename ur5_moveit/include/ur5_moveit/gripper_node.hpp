#ifndef GRIPPER_NODE_HPP_
#define GRIPPER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <memory>
#include <chrono>
#include <string>

#include "ur5_moveit/srv/gripper_request.hpp" 

class GripperNode : public rclcpp::Node
{
public:
    GripperNode();
    void init_moveit();

private:
    /**
     * @brief Service callback
     * It receives open or close commands and executes them using MoveIt
     */
    void handle_gripper_command(
        const std::shared_ptr<ur5_moveit::srv::GripperRequest::Request> request,
        std::shared_ptr<ur5_moveit::srv::GripperRequest::Response> response);

    // service server
    rclcpp::Service<ur5_moveit::srv::GripperRequest>::SharedPtr service_;
    
    // MoveIt interface
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
};

#endif // GRIPPER_NODE_HPP_