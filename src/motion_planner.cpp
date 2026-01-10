#include "group18_assignment_2/motion_planner.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp> 

MotionPlanner::MotionPlanner() : Node("motion_planner_node")
{
    // Action server
    action_server_ = rclcpp_action::create_server<MoveToPose>(this, "move_to_pose",
        std::bind(&MotionPlanner::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MotionPlanner::handleCancel, this, std::placeholders::_1),
        std::bind(&MotionPlanner::handleAccepted, this, std::placeholders::_1));
        
    RCLCPP_INFO(this->get_logger(), "Motion Planner Action Server started");
}

void MotionPlanner::initializeMoveIt()
{
    static const std::string PLANNING_GROUP = "ir_arm";
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
    
    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);
    move_group_->setPlanningTime(20.0);

    move_group_->setStartStateToCurrentState();
    
    RCLCPP_INFO(this->get_logger(), "Motion Planner ready");
    
    geometry_msgs::msg::PoseStamped p = move_group_->getCurrentPose();
    RCLCPP_INFO(this->get_logger(), "Current Pose frame id: %s", p.header.frame_id.c_str());
}

rclcpp_action::GoalResponse MotionPlanner::handleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveToPose::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received target pose: [%.2f, %.2f, %.2f]", 
        goal->target_pose.pose.position.x,
        goal->target_pose.pose.position.y,
        goal->target_pose.pose.position.z);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionPlanner::handleCancel(const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_WARN(this->get_logger(), "Cancel requested");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionPlanner::handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    std::thread(&MotionPlanner::execute, this, goal_handle).detach();
}

void MotionPlanner::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveToPose::Feedback>();
    auto result = std::make_shared<MoveToPose::Result>();
        
    if (!move_group_) {
        result->success = false;
        result->message = "MoveIt not initialized";
        goal_handle->abort(result);
        return;
    }

    // Planning
    feedback->status = "Planning trajectory...";
    goal_handle->publish_feedback(feedback);
        
    move_group_->setPoseTarget(goal->target_pose);
    
    auto plan_result = move_group_->plan(plan_);
        
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
        result->success = false;
        // [FIXED] Use errorCodeToString instead of error_code_to_string
        result->message = "Planning failed: " + std::string(moveit::core::errorCodeToString(plan_result));
        
        RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        
        goal_handle->abort(result);
        return;
    }

    // Executing
    feedback->status = "Executing motion...";
    goal_handle->publish_feedback(feedback);
    
    std::atomic<bool> execution_done{false};
    moveit::core::MoveItErrorCode exec_code; 
    
    auto execution_thread = std::thread([this, &execution_done, &exec_code]() {
        exec_code = move_group_->execute(plan_);
        execution_done = true;
    });
    
    // if during the execution a cancel is requested
    while (!execution_done) {
        if (goal_handle->is_canceling()) {
            RCLCPP_WARN(this->get_logger(), "Stopping robot");
            move_group_->stop();
            execution_thread.join();
            
            result->success = false;
            result->message = "Goal cancelled";
            goal_handle->canceled(result);
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    execution_thread.join();
    
    // Result
    if (exec_code == moveit::core::MoveItErrorCode::SUCCESS) {
        result->success = true;
        result->message = "Motion completed";
        goal_handle->succeed(result);
    } else {
        result->success = false;
        result->message = "Execution failed: " + std::string(moveit::core::errorCodeToString(exec_code));
        RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
        goal_handle->abort(result);
    }    
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionPlanner>();
    
    // Initialize MoveIt after node creation
    node->initializeMoveIt();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}