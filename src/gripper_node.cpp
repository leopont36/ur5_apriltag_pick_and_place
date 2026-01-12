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
    // Execute in a separate thread so we don't block the executor
    std::thread{std::bind(&GripperNode::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void GripperNode::execute(const std::shared_ptr<GoalHandleGripper> goal_handle)
{
    if (!gripper_group_) init_moveit();

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<GripperAction::Result>();
    
    if (goal->command == "open") {
        RCLCPP_INFO(this->get_logger(), "Eseguendo OPEN (Standard)...");
        
        bool setup = gripper_group_->setNamedTarget("open");
        if (!setup) {
            result->success = false;
            result->message = "Target open non valido";
            goal_handle->abort(result);
            return;
        }

        auto error_code = gripper_group_->move();
        
        if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
            result->success = true;
            result->message = "Apertura completata";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "OPEN finita pulita.");

            /*if(!goal->object_id.empty()) {
                RCLCPP_INFO(this->get_logger(), "Detaching object '%s' from gripper.", goal->object_id.c_str());
                gripper_group_->detachObject(goal->object_id);
            }*/
        } else {
            result->success = false;
            result->message = "Errore apertura MoveIt";
            goal_handle->abort(result);
        }
        return; 
    }

    else if (goal->command == "close") {
        RCLCPP_INFO(this->get_logger(), "Eseguendo CLOSE (Forzata 2s)...");

        if (!gripper_group_->setJointValueTarget(DRIVER_JOINT_NAME, 0.75)) {
            result->success = false;
            result->message = "Target close non valido";
            goal_handle->abort(result);

            /*if(!goal->object_id.empty()) {
                RCLCPP_INFO(this->get_logger(), "Attaching object '%s' to gripper.", goal->object_id.c_str());
                gripper_group_->attachObject(goal->object_id, "tool0", gripper_group_->getLinkNames());
            }*/

            return;
        }

        gripper_group_->asyncMove();
        std::this_thread::sleep_for(std::chrono::seconds(3));

        gripper_group_->stop();

        result->success = true;
        result->message = "Chiusura forzata (Presa)";
        goal_handle->succeed(result);
        
        RCLCPP_INFO(this->get_logger(), "CLOSE forzata completata.");
    }
    else {
        result->success = false;
        result->message = "Comando sconosciuto";
        goal_handle->abort(result);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperNode>();
    
    // MultiThreadedExecutor is best for Actions
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}