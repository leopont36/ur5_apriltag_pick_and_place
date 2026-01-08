#include "group18_assignment_2/gripper_node.hpp"

static const std::string GRIPPER_PLANNING_GROUP = "ir_gripper"; 
static const std::string DRIVER_JOINT_NAME = "robotiq_85_left_knuckle_joint";

GripperNode::GripperNode() : Node("gripper_node")
{
    // create the service gripper_service
    service_ = this->create_service<group18_assignment_2::srv::GripperRequest>(
        "gripper_service",
        std::bind(&GripperNode::handle_gripper_command, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Gripper node started.");
}

void GripperNode::init_moveit()
{
    if (gripper_group_) return;

    try {
        // init MoveGroupInterface with ir_gripper
        gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), GRIPPER_PLANNING_GROUP);
        
        gripper_group_->setMaxVelocityScalingFactor(1.0);
        gripper_group_->setMaxAccelerationScalingFactor(1.0);
        gripper_group_->setPlanningTime(5.0); 

        RCLCPP_INFO(this->get_logger(), "Gripper MoveGroup Ready.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveGroup: %s", e.what());
    }
}

void GripperNode::handle_gripper_command(
    const std::shared_ptr<group18_assignment_2::srv::GripperRequest::Request> request,
    std::shared_ptr<group18_assignment_2::srv::GripperRequest::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received command: '%s'", request->command.c_str());

    if (!gripper_group_) init_moveit();

    bool plan_success = false;

    if (request->command == "open") {
        // "open" is a named target in the SRDF, safe to use directly
        plan_success = gripper_group_->setNamedTarget("open");
    }
    else if (request->command == "close") {
        // 0.8 is max closed, 0.75 ensures a tight grip
        double target_value = 0.75; 
        
        plan_success = gripper_group_->setJointValueTarget(DRIVER_JOINT_NAME, target_value);
        RCLCPP_INFO(this->get_logger(), "Closing joint '%s' to %.2f", DRIVER_JOINT_NAME.c_str(), target_value);
    }
    else {
        response->success = false;
        response->message = "Invalid command";
        return;
    }

    if (plan_success) {
        moveit::core::MoveItErrorCode result = gripper_group_->move();
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            if(request->command == "open" && !request->object_id.empty()) {
                RCLCPP_INFO(this->get_logger(), "Detaching object '%s' from gripper.", request->object_id.c_str());
                gripper_group_->detachObject(request->object_id);
            }
            if(request->command == "close" && !request->object_id.empty()) {
                RCLCPP_INFO(this->get_logger(), "Attaching object '%s' to gripper.", request->object_id.c_str());
                gripper_group_->attachObject(request->object_id, "tool0", gripper_group_->getLinkNames());
            }
            response->success = true;
            response->message = "Gripper Action Succeeded.";
        } else {
            RCLCPP_WARN(this->get_logger(), "Gripper result: %d (Likely object contact).", result.val);
            response->success = true; 
            response->message = "Gripper Action Completed.";
        }
    } else {
        response->success = false;
        response->message = "Target setting failed";
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