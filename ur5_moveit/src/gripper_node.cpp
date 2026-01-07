#include "ur5_moveit/gripper_node.hpp"

// Defined based on your RViz screenshot verification
static const std::string GRIPPER_PLANNING_GROUP = "ir_gripper"; 

GripperNode::GripperNode() : Node("gripper_node")
{
    // create the service gripper_service
    service_ = this->create_service<ur5_moveit::srv::GripperRequest>(
        "gripper_service",
        std::bind(&GripperNode::handle_gripper_command, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Gripper node started.");
}

void GripperNode::init_moveit()
{
    try {
        // init MoveGroupInterface with ir_gripper
        gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), GRIPPER_PLANNING_GROUP);
        
        gripper_group_->setMaxVelocityScalingFactor(1.0);
        gripper_group_->setMaxAccelerationScalingFactor(1.0);
        gripper_group_->setPlanningTime(5.0); 
        
        RCLCPP_INFO(this->get_logger(), "Gripper MoveGroup Ready.");
        
        // log available targets (open, close)
        auto targets = gripper_group_->getNamedTargets();
        for(const auto& t : targets) {
            RCLCPP_INFO(this->get_logger(), " - Found target: %s", t.c_str());
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveGroup: %s", e.what());
    }
}

void GripperNode::handle_gripper_command(
    const std::shared_ptr<ur5_moveit::srv::GripperRequest::Request> request,
    std::shared_ptr<ur5_moveit::srv::GripperRequest::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received command: '%s'", request->command.c_str());

    if (!gripper_group_) {
        init_moveit();
    }

    if (!gripper_group_) {
        response->success = false;
        response->message = "Gripper MoveGroup not initialized yet.";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    // check that the request command is valid
    bool target_found = gripper_group_->setNamedTarget(request->command);

    if (!target_found) {
        response->success = false;
        response->message = "Target command does not exist (Try 'open' or 'close').";
        RCLCPP_ERROR(this->get_logger(), "Invalid target: %s", request->command.c_str());
        return;
    }

    // plan and execute the move
    moveit::core::MoveItErrorCode result = gripper_group_->move();

    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Gripper Action Succeeded.");
    } else {
        response->success = false;
        RCLCPP_ERROR(this->get_logger(), "Gripper Action Failed (Error Code: %d)", result.val);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<GripperNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}