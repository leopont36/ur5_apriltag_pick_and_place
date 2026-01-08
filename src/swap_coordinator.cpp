#include "group18_assignment_2/swap_coordinator.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

SwapCoordinator::SwapCoordinator() : Node("swap_coordinator_node")
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    action_client_ = rclcpp_action::create_client<MoveToPose>(this, "move_to_pose");
    // Removed blocking wait in constructor to avoid locking up if server isn't ready immediately
    RCLCPP_INFO(this->get_logger(), "Swap Coordinator initialized");
    
    gripper_client_ = this->create_client<group18_assignment_2::srv::GripperRequest>("gripper_service");
    RCLCPP_INFO(this->get_logger(), "Swap Coordinator initialized");
}

bool SwapCoordinator::swapTags(const std::string& tag1_frame, const std::string& tag2_frame)
{
    // wait for action server here to ensure connection
    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available");
        return false;
    }

    geometry_msgs::msg::PoseStamped grasp1, grasp2, temp;
    
    // get grasp poses
    if (!getGraspPose(tag1_frame, grasp1)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get grasp pose for %s", tag1_frame.c_str());
        return false;
    }
    if (!getGraspPose(tag2_frame, grasp2)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get grasp pose for %s", tag2_frame.c_str());
        return false;
    }

    // Temporary position offset along X
    temp = grasp2;
    temp.pose.position.x += 0.15;

    RCLCPP_INFO(get_logger(), "Grasp1: [%.3f, %.3f, %.3f]", grasp1.pose.position.x, grasp1.pose.position.y, grasp1.pose.position.z);
    RCLCPP_INFO(get_logger(), "Grasp2: [%.3f, %.3f, %.3f]", grasp2.pose.position.x, grasp2.pose.position.y, grasp2.pose.position.z);
    RCLCPP_INFO(get_logger(), "Temp:   [%.3f, %.3f, %.3f]", temp.pose.position.x, temp.pose.position.y, temp.pose.position.z);

    // Sequence for swapping
    RCLCPP_INFO(get_logger(), "Moving cube 1 to temp position");
    if (!pickAndPlace(grasp1, temp, tag1_frame))
        return false;

    RCLCPP_INFO(get_logger(), "Moving cube 2 to position 1");
    if (!pickAndPlace(grasp2, grasp1, tag2_frame))
        return false;

    RCLCPP_INFO(get_logger(), "Moving cube 1 to position 2");
    if (!pickAndPlace(temp, grasp2, tag1_frame))
        return false;

    RCLCPP_INFO(this->get_logger(), "Successfully swapped %s and %s", tag1_frame.c_str(), tag2_frame.c_str());
    return true;
}

bool SwapCoordinator::getGraspPose(const std::string& tag_frame, geometry_msgs::msg::PoseStamped& grasp_pose)
{
    try {
        RCLCPP_INFO(this->get_logger(), "Computing grasp pose for %s", tag_frame.c_str());
        
        // wait for transform to be available
        rclcpp::Time now = this->get_clock()->now();
        // Use TimePointZero to get latest transform available without strict time sync issues
        if (!tf_buffer_->canTransform("base_link", tag_frame, tf2::TimePointZero, 
            std::chrono::seconds(20))) {
            RCLCPP_ERROR(this->get_logger(), 
                "Transform not available for %s after 20s", tag_frame.c_str());
            return false;
        }
        
        // read the transform of the tag
        auto tag_tf = tf_buffer_->lookupTransform("base_link", tag_frame, 
            tf2::TimePointZero);
        
        grasp_pose.header.frame_id = "base_link";
        grasp_pose.header.stamp = tag_tf.header.stamp;

        // Tuning offsets to align physics with planning
        double offset_x = 0.02;
        double offset_y = 0;
        double offset_z = 0;
        
        grasp_pose.pose.position.x = tag_tf.transform.translation.x + offset_x;
        grasp_pose.pose.position.y = tag_tf.transform.translation.y + offset_y;
        grasp_pose.pose.position.z = tag_tf.transform.translation.z + offset_z;

        // Obtain the tag's orientation
        tf2::Quaternion tag_quat(
            tag_tf.transform.rotation.x,
            tag_tf.transform.rotation.y,
            tag_tf.transform.rotation.z,
            tag_tf.transform.rotation.w
        );
        
        // Rotate the gripper 180° around X to face downwards
        tf2::Quaternion flip_x;
        flip_x.setRPY(M_PI, 0, 0);  
        
        tf2::Quaternion gripper_quat = tag_quat * flip_x;
        gripper_quat.normalize();
        
        grasp_pose.pose.orientation = tf2::toMsg(gripper_quat);
        
        RCLCPP_INFO(this->get_logger(), "Grasp pose: [%.3f, %.3f, %.3f] | quat: [%.2f, %.2f, %.2f, %.2f]",
            grasp_pose.pose.position.x, grasp_pose.pose.position.y, grasp_pose.pose.position.z, gripper_quat.x(), gripper_quat.y(), gripper_quat.z(), gripper_quat.w());
        return true;
        
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "TF error for %s: %s", tag_frame.c_str(), ex.what());
        return false;
    }
}

bool SwapCoordinator::moveArmOverTarget(geometry_msgs::msg::PoseStamped pose, double offset_z)
{
    pose.pose.position.z += offset_z;
    
    auto goal = MoveToPose::Goal();
    goal.target_pose = pose;
    
    auto send_opts = rclcpp_action::Client<MoveToPose>::SendGoalOptions();
    auto goal_future = action_client_->async_send_goal(goal, send_opts);
    
    if (goal_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "Failed to send move goal (timeout)");
        return false;
    }
    
    auto goal_handle = goal_future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "Goal rejected");
        return false;
    }
    
    auto result_future = action_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "Failed to get result (timeout)");
        return false;
    }
    
    auto result = result_future.get();
    if (!result.result->success) {
        RCLCPP_ERROR(get_logger(), "Move failed: %s", result.result->message.c_str());
        return false;
    }
    
    return true;
}

bool SwapCoordinator::controlGripper(const std::string& cmd, const std::string& object_id)
{
    RCLCPP_INFO(get_logger(), "Gripper: %s", cmd.c_str());
    auto req = std::make_shared<group18_assignment_2::srv::GripperRequest::Request>();
    req->command = cmd;
    req->object_id = object_id;
        
    auto future = gripper_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(15)) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "Gripper service timeout");
        return false;
    }
        
    auto response = future.get();
    if (!response->success) {
        // Log warning but continue if it's just a grasp contact
        RCLCPP_WARN(get_logger(), "Gripper reported: %s", response->message.c_str());
    }
        
    return true;
}

bool SwapCoordinator::pickAndPlace(geometry_msgs::msg::PoseStamped pick, geometry_msgs::msg::PoseStamped place, const std::string& object_id)
{
    // Settings to avoid collision/air-grasp
    double hover_z = 0.15; 
    
    RCLCPP_INFO(get_logger(), "Pick: [%.3f, %.3f, %.3f] → Place: [%.3f, %.3f, %.3f]", pick.pose.position.x, pick.pose.position.y, pick.pose.position.z, place.pose.position.x, place.pose.position.y, place.pose.position.z);
    if (!controlGripper("open"))
        return false; 
    
    // PICK SEQUENCE
    RCLCPP_INFO(get_logger(), "Approaching (hover)...");
    if (!moveArmOverTarget(pick, hover_z)) return false;
    rclcpp::sleep_for(std::chrono::milliseconds(800));

    RCLCPP_INFO(get_logger(), "Closing gripper");
    if (!controlGripper("close", object_id)) 
        return false;
    
    RCLCPP_INFO(get_logger(), "Lifting the cube");
    if (!moveArmOverTarget(pick, hover_z)) 
        return false;
        
    RCLCPP_INFO(get_logger(), "Moving to place position");
    if (!moveArmOverTarget(place, hover_z))
        return false;
        
    RCLCPP_INFO(get_logger(), "Opening gripper");
    if (!controlGripper("open", object_id)) 
        return false;
    
    RCLCPP_INFO(get_logger(), "Retreating");
    if (!moveArmOverTarget(place, hover_z))
        return false;
    
    RCLCPP_INFO(get_logger(), "Pick-and-place complete!");
    return true;
}

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SwapCoordinator>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() {
        executor.spin();
    });

    RCLCPP_INFO(node->get_logger(), "Warming up TF listener...");
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    if (node->swapTags("tag36h11:10", "tag36h11:1")) 
    {
        RCLCPP_INFO(node->get_logger(), "swap completed!");
    } else {
        RCLCPP_ERROR(node->get_logger(), "swap failed");
    }
    
    executor.cancel();
    if (spinner.joinable()) {
        spinner.join();
    }
    rclcpp::shutdown();
    return 0;
}