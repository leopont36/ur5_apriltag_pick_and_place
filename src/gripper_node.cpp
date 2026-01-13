#include "group18_assignment_2/gripper_node.hpp"

static const std::string GRIPPER_PLANNING_GROUP = "ir_gripper"; 
static const std::string DRIVER_JOINT_NAME = "robotiq_85_left_knuckle_joint";

GripperNode::GripperNode() : Node("gripper_node")
{
    this->action_server_ = rclcpp_action::create_server<GripperAction>(
      this,
      "gripper_action",
      std::bind(&GripperNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GripperNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&GripperNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Gripper Action Server ready.");
}

void GripperNode::init_moveit()
{
    if (gripper_group_) return;
    try {
        gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), GRIPPER_PLANNING_GROUP);
        
        gripper_group_->setMaxVelocityScalingFactor(1.0);
        gripper_group_->setMaxAccelerationScalingFactor(1.0);

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        
        std::vector<std::string> gripper_links = {
            "tool0",
            "ur_to_robotiq_link", 
            "gripper_mount_link",
            "robotiq_85_base_link",
            "robotiq_85_left_knuckle_link",
            "robotiq_85_right_knuckle_link",
            "robotiq_85_left_finger_link",
            "robotiq_85_right_finger_link"
        };
        
        RCLCPP_INFO(this->get_logger(), "Gripper MoveGroup initialized.");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "MoveIt init failed: %s", e.what());
    }
}

rclcpp_action::GoalResponse GripperNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GripperAction::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal: %s", goal->command.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperNode::handle_cancel(
    const std::shared_ptr<GoalHandleGripper> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Goal cancelled");
    if (gripper_group_) gripper_group_->stop();
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperNode::handle_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle)
{
    std::thread{std::bind(&GripperNode::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void GripperNode::execute(const std::shared_ptr<GoalHandleGripper> goal_handle)
{
    if (!gripper_group_) init_moveit();

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<GripperAction::Result>();
    
    if (goal->command == "open") {
        RCLCPP_INFO(this->get_logger(), "Trying to open ...");
        
        bool setup = gripper_group_->setNamedTarget("open");
        if (!setup) {
            result->success = false;
            result->message = "Target open not valid";
            goal_handle->abort(result);
            return;
        }

        auto error_code = gripper_group_->move();
        
        if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = true;
            result->message = "Open completed";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "OPEN finished succesfully.");
        } else {
            result->success = false;
            result->message = "Error open MoveIt";
            goal_handle->abort(result);
        }
        return; 
    }

    else if (goal->command == "grasp") {
        RCLCPP_INFO(this->get_logger(), "Trying to grasp...");

        if (!gripper_group_->setJointValueTarget(DRIVER_JOINT_NAME, 0.75)) {
            result->success = false;
            result->message = "Target grasp not valid";
            goal_handle->abort(result);
        }

        gripper_group_->asyncMove();
        std::this_thread::sleep_for(std::chrono::seconds(3));

        gripper_group_->stop();

        result->success = true;
        result->message = "Grasp Completed";
        goal_handle->succeed(result);
        
        RCLCPP_INFO(this->get_logger(), "GRASP finished succesfully.");
    }
    else if (goal->command == "close") {
        RCLCPP_INFO(this->get_logger(), "Trying to close ...");
        
        bool setup = gripper_group_->setNamedTarget("close");
        if (!setup) {
            result->success = false;
            result->message = "Target close not valid";
            goal_handle->abort(result);
            return;
        }

        auto error_code = gripper_group_->move();
        
        if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = true;
            result->message = "Close completed";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "CLOSE finished succesfully.");
        } else {
            result->success = false;
            result->message = "Error close MoveIt";
            goal_handle->abort(result);
        }
        return; 
    }
    else {
        result->success = false;
        result->message = "Unknown command";
        goal_handle->abort(result);
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