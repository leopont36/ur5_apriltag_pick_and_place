#ifndef GRIPPER_NODE_HPP_
#define GRIPPER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <memory>
#include <chrono>
#include <string>

#include "group18_assignment_2/srv/gripper_request.hpp" 

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
        const std::shared_ptr<group18_assignment_2::srv::GripperRequest::Request> request,
        std::shared_ptr<group18_assignment_2::srv::GripperRequest::Response> response);

    // service server
    rclcpp::Service<group18_assignment_2::srv::GripperRequest>::SharedPtr service_;
    
    // MoveIt interface
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
};

#endif // GRIPPER_NODE_HPP_