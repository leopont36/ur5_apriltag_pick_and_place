#include "group18_assignment_2/motion_planner.hpp"

MotionPlanner::MotionPlanner() : Node("motion_planner_node")
{
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
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);
    move_group_->setPlanningTime(30.0);
    move_group_->setNumPlanningAttempts(15);
    move_group_->setStartStateToCurrentState();
    RCLCPP_INFO(this->get_logger(), "Motion Planner ready");
    geometry_msgs::msg::PoseStamped p = move_group_->getCurrentPose();
    RCLCPP_INFO(this->get_logger(), "Current Pose frame id: %s", p.header.frame_id.c_str());
}

rclcpp_action::GoalResponse MotionPlanner::handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveToPose::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received target pose: [%.2f, %.2f, %.2f]",goal->target_pose.pose.position.x, goal->target_pose.pose.position.y, goal->target_pose.pose.position.z);
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

    if (!move_group_)
    {
        result->success = false;
        result->message = "MoveIt not initialized";
        goal_handle->abort(result);
        return;
    }

    if(goal->message == "FINISH")
    {
        RCLCPP_INFO(this->get_logger(), "Received finish command, moving to zero pose");
        MoveToZeroPose();
        result->success = true;
        result->message = "Finished";
        goal_handle->succeed(result);
        return;
    }

    const int MAX_RETRIES = 2;
    bool use_constraints = goal->constrain_orientation;

    for (int attempt = 0; attempt < MAX_RETRIES; ++attempt)
    {
        // Retry logic: go to safety pose and disable constraints
        if (attempt > 0)
        {
            RCLCPP_WARN(this->get_logger(), "Retry attempt %d/%d", attempt + 1, MAX_RETRIES);
            feedback->status = "Moving to safety pose before retry...";
            goal_handle->publish_feedback(feedback);
            SetSafetyPose();
            use_constraints = false;
            RCLCPP_INFO(this->get_logger(), "Constraints disabled for retry");
        }

        // Planning phase
        feedback->status = attempt == 0 ? "Planning trajectory..." : "Replanning trajectory...";
        goal_handle->publish_feedback(feedback);
        move_group_->setPoseTarget(goal->target_pose);

        if (use_constraints)
        {
            RCLCPP_INFO(this->get_logger(), "Enforcing Upright Constraints");
            moveit_msgs::msg::Constraints path_constraints;
            moveit_msgs::msg::OrientationConstraint ocm;
            ocm.link_name = "tool0";
            ocm.header.frame_id = "base_link";
            ocm.orientation = goal->target_pose.pose.orientation;
            ocm.absolute_x_axis_tolerance = 0.1;
            ocm.absolute_y_axis_tolerance = 0.1;
            ocm.absolute_z_axis_tolerance = M_PI;
            ocm.weight = 1.0;
            path_constraints.orientation_constraints.push_back(ocm);
            move_group_->setPathConstraints(path_constraints);
        }

        auto plan_result = move_group_->plan(plan_);
        if (use_constraints) move_group_->clearPathConstraints();

        if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed (attempt %d/%d): %s", 
                        attempt + 1, MAX_RETRIES, moveit::core::errorCodeToString(plan_result).c_str());
            if (attempt < MAX_RETRIES - 1) continue;
            result->success = false;
            result->message = "Planning failed after " + std::to_string(MAX_RETRIES) + " attempts";
            goal_handle->abort(result);
            return;
        }

        // Execution phase
        feedback->status = "Executing motion...";
        goal_handle->publish_feedback(feedback);

        std::atomic<bool> execution_done{false};
        moveit::core::MoveItErrorCode exec_code;
        auto execution_thread = std::thread([this, &execution_done, &exec_code]() {
            exec_code = move_group_->execute(plan_);
            execution_done = true;
        });

        // Handle cancellation during execution
        while (!execution_done)
        {
            if (goal_handle->is_canceling())
            {
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

        // Check execution result
        if (exec_code == moveit::core::MoveItErrorCode::SUCCESS)
        {
            result->success = true;
            result->message = attempt > 0 ? "Motion completed (after retry)" : "Motion completed";
            goal_handle->succeed(result);
            return;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Execution failed (attempt %d/%d): %s", 
                        attempt + 1, MAX_RETRIES, moveit::core::errorCodeToString(exec_code).c_str());
            if (attempt < MAX_RETRIES - 1) continue;
            result->success = false;
            result->message = "Execution failed after " + std::to_string(MAX_RETRIES) + " attempts";
            goal_handle->abort(result);
            return;
        }
    }
}

void MotionPlanner::SetSafetyPose()
{
    if(!move_group_)
    {
        RCLCPP_ERROR(this->get_logger(), "MoveIt not initialized");
        return;
    }

    std::vector<double> joint_values = move_group_->getCurrentJointValues();
    double angle = normalize_rad(joint_values[3]);
    std::vector<double> safe_values = (angle > 0.0) ? SAFETY_POSE_POS : SAFETY_POSE_NEG;
    
    safe_values[5] = joint_values[5]; 
    safe_values[4] = joint_values[4]; 
    safe_values[3] = joint_values[3]; 

    move_group_->setJointValueTarget(safe_values);

    auto plan_result = move_group_->plan(plan_);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Planning to safety pose failed: %s", moveit::core::errorCodeToString(plan_result).c_str());
        return;     
    }
    
    auto exec_code = move_group_->execute(plan_);
    if (exec_code != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Execution to safety pose failed: %s", moveit::core::errorCodeToString(exec_code).c_str());
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Successfully moved to safety pose");
}

void MotionPlanner::MoveToZeroPose()
{
    if(!move_group_)
    {
        RCLCPP_ERROR(this->get_logger(), "MoveIt not initialized");
        return;
    }

    move_group_->setJointValueTarget(ZERO_POSE);

    auto plan_result = move_group_->plan(plan_);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Planning to zero pose failed: %s", moveit::core::errorCodeToString(plan_result).c_str());
        return;     
    }
    
    auto exec_code = move_group_->execute(plan_);
    if (exec_code != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Execution to zero pose failed: %s", moveit::core::errorCodeToString(exec_code).c_str());
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Successfully moved to zero pose");
}



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionPlanner>();

    node->initializeMoveIt();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}