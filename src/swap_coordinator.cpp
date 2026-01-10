#include "group18_assignment_2/swap_coordinator.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <cmath> // For M_PI_2

SwapCoordinator::SwapCoordinator() : Node("swap_coordinator_node")
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Init Visualizer for debugging
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    action_client_ = rclcpp_action::create_client<MoveToPose>(this, "move_to_pose");
    gripper_client_ = this->create_client<group18_assignment_2::srv::GripperRequest>("gripper_service");
    
    RCLCPP_INFO(this->get_logger(), "Swap Coordinator initialized (RIGOROUS SIDE GRASP MODE)");
}

// Helper: Calculates the position X meters "backwards" from the target pose
// Uses the Z-axis of the gripper as the approach vector.
geometry_msgs::msg::PoseStamped SwapCoordinator::computeApproachPose(const geometry_msgs::msg::PoseStamped& target, double dist)
{
    geometry_msgs::msg::PoseStamped approach = target;

    // Convert quaternion to rotation matrix
    tf2::Quaternion q(
        target.pose.orientation.x, target.pose.orientation.y,
        target.pose.orientation.z, target.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    
    // In standard robotic gripper frames, Z is the approach vector (pointing out of the palm)
    tf2::Vector3 approach_vector = m.getColumn(2);

    // We move NEGATIVE distance along the Z vector to back up
    approach.pose.position.x -= approach_vector.getX() * dist;
    approach.pose.position.y -= approach_vector.getY() * dist;
    approach.pose.position.z -= approach_vector.getZ() * dist;

    return approach;
}

bool SwapCoordinator::swapTags(const std::string& tag1_frame, const std::string& tag2_frame)
{
    // wait for action server here to ensure connection
    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available");
        return false;
    }

    // Safety: Ensure gripper is open 
    controlGripper("open", tag1_frame);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    geometry_msgs::msg::PoseStamped grasp1, grasp2, temp;
    
    // Calculate poses using the rigorous getGraspPose function
    if (!getGraspPose(tag1_frame, grasp1)) return false;
    if (!getGraspPose(tag2_frame, grasp2)) return false;

    // --- TEMP POSITION CALCULATION ---
    // Midpoint between the two tags, but pushed back in Y for safety
    // We maintain the Z-height and Orientation of grasp2 for consistency
    temp = grasp2;
    temp.pose.position.x = (grasp1.pose.position.x + grasp2.pose.position.x) / 2.0;
    temp.pose.position.y = -0.40; // 40cm back from robot center (safe zone on table)

    // --- SWAP SEQUENCE ---

    RCLCPP_INFO(get_logger(), ">>> PHASE 1: Cube 1 -> Temp");
    if (!pickAndPlace(grasp1, temp, tag1_frame)) return false;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Refreshed grasp2 calculation in case the robot shifted slightly (optional but rigorous)
    if (!getGraspPose(tag2_frame, grasp2)) return false;

    RCLCPP_INFO(get_logger(), ">>> PHASE 2: Cube 2 -> Pos 1");
    if (!pickAndPlace(grasp2, grasp1, tag2_frame)) return false;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(get_logger(), ">>> PHASE 3: Cube 1 -> Pos 2");
    // Note: temp is a coordinate, not a tracked object, so we don't re-query it
    if (!pickAndPlace(temp, grasp2, tag1_frame)) return false;

    RCLCPP_INFO(this->get_logger(), "SWAP COMPLETED SUCCESSFULLY");
    return true;
}

bool SwapCoordinator::getGraspPose(const std::string& tag_frame, geometry_msgs::msg::PoseStamped& grasp_pose)
{
    try {
        if (!tf_buffer_->canTransform("base_link", tag_frame, tf2::TimePointZero, std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Transform not available for %s", tag_frame.c_str());
            return false;
        }
        
        auto tag_tf = tf_buffer_->lookupTransform("base_link", tag_frame, tf2::TimePointZero);
        
        
        tf2::Quaternion tag_quat(
            tag_tf.transform.rotation.x, tag_tf.transform.rotation.y,
            tag_tf.transform.rotation.z, tag_tf.transform.rotation.w);
        
        double roll, pitch, yaw;
        tf2::Matrix3x3(tag_quat).getRPY(roll, pitch, yaw);

        tf2::Quaternion final_q;
        // orientation aligned with yaw, with gripper orizontal
        final_q.setRPY(-M_PI_2, 0, yaw);
        final_q.normalize();

        // gripper length offset (wrt arm)
        const double GRIPPER_LENGTH = 0.16296; 

        // get the direction the gripper is pointing
        tf2::Matrix3x3 m(final_q);
        tf2::Vector3 gripper_z = m.getColumn(2);
        

        double offset_x = 0.0; 
        double offset_y = 0.0;
        double offset_z = 0.0; //????
        
        if (tag_frame == "tag36h11:1") { // RED CUBE 
            offset_x = 0.0302; 
            offset_y = 0.0054; 
        } 
        else if (tag_frame == "tag36h11:10") { // BLUE CUBE 
            offset_x = 0.0349; 
            offset_y = 0.0085;
        }

        // define target tip position
        double target_tip_x = tag_tf.transform.translation.x + offset_x;
        double target_tip_y = tag_tf.transform.translation.y + offset_y;
        double target_tip_z = tag_tf.transform.translation.z + offset_z; 

        // calculate the wrist position wrist = tip - (direction * length)
        grasp_pose.header.frame_id = "base_link";
        grasp_pose.header.stamp = this->get_clock()->now();

        grasp_pose.pose.position.x = target_tip_x - (gripper_z.getX() * GRIPPER_LENGTH);
        grasp_pose.pose.position.y = target_tip_y - (gripper_z.getY() * GRIPPER_LENGTH);
        grasp_pose.pose.position.z = target_tip_z - (gripper_z.getZ() * GRIPPER_LENGTH);
        
        grasp_pose.pose.orientation = tf2::toMsg(final_q);

        // debug visualization
        geometry_msgs::msg::TransformStamped t;
        t.header = grasp_pose.header;
        t.child_frame_id = "debug_wrist_" + tag_frame;
        t.transform.translation.x = grasp_pose.pose.position.x;
        t.transform.translation.y = grasp_pose.pose.position.y;
        t.transform.translation.z = grasp_pose.pose.position.z;
        t.transform.rotation = grasp_pose.pose.orientation;
        tf_broadcaster_->sendTransform(t);

        return true;
        
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "TF error: %s", ex.what());
        return false;
    }
}

bool SwapCoordinator::moveArmOverTarget(geometry_msgs::msg::PoseStamped pose)
{
    auto goal = MoveToPose::Goal();
    goal.target_pose = pose;
    
    auto send_opts = rclcpp_action::Client<MoveToPose>::SendGoalOptions();
    auto goal_future = action_client_->async_send_goal(goal, send_opts);
    
    if (goal_future.wait_for(std::chrono::seconds(20)) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "Move action server timed out");
        return false;
    }
    
    auto goal_handle = goal_future.get();
    if (!goal_handle) {
         RCLCPP_ERROR(get_logger(), "Move goal rejected");
         return false;
    }
    
    auto result_future = action_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready) return false;
    
    auto result = result_future.get();
    if (!result.result->success) {
        RCLCPP_WARN(get_logger(), "Move warning/error: %s", result.result->message.c_str());
        // We return false, but depending on the error, you might want to retry
        return false;
    }
    return true;
}

bool SwapCoordinator::controlGripper(const std::string& cmd, const std::string& object_id)
{
    RCLCPP_INFO(get_logger(), "Gripper Command: %s", cmd.c_str());
    auto req = std::make_shared<group18_assignment_2::srv::GripperRequest::Request>();
    req->command = cmd;
    req->object_id = object_id;
        
    auto future = gripper_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "Gripper service failed/timeout");
        return false;
    }

    future.get(); 
    return true;
}

bool SwapCoordinator::pickAndPlace(geometry_msgs::msg::PoseStamped pick, geometry_msgs::msg::PoseStamped place, const std::string& object_id)
{
    // Approach distance: 15cm back from the cube
    double approach_dist = 0.05;

    // Calculate Approach Poses
    geometry_msgs::msg::PoseStamped pick_approach = computeApproachPose(pick, approach_dist);
    geometry_msgs::msg::PoseStamped place_approach = computeApproachPose(place, approach_dist);

    //  ensure open
    controlGripper("open", object_id);

    // PICK
    RCLCPP_INFO(get_logger(), "[PICK] Moving to Approach...");
    if (!moveArmOverTarget(pick_approach)) return false;

    RCLCPP_INFO(get_logger(), "[PICK] Moving to Grasp...");
    if (!moveArmOverTarget(pick)) return false;
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_INFO(get_logger(), "[PICK] Closing Gripper...");
    if (!controlGripper("close", object_id)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_INFO(get_logger(), "[PICK] Retreating...");
    if (!moveArmOverTarget(pick_approach)) return false;

    // PLACE
    RCLCPP_INFO(get_logger(), "[PLACE] Moving to Approach...");
    if (!moveArmOverTarget(place_approach)) return false;

    RCLCPP_INFO(get_logger(), "[PLACE] Placing...");
    if (!moveArmOverTarget(place)) return false;

    RCLCPP_INFO(get_logger(), "[PLACE] Releasing...");
    if (!controlGripper("open", object_id)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_INFO(get_logger(), "[PLACE] Retreating...");
    if (!moveArmOverTarget(place_approach)) return false;

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

    RCLCPP_INFO(node->get_logger(), "Waiting for TF buffer to fill...");
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    if (node->swapTags("tag36h11:10", "tag36h11:1")) 
    {
        RCLCPP_INFO(node->get_logger(), "--- SWAP SUCCESS ---");
    } else {
        RCLCPP_ERROR(node->get_logger(), "--- SWAP FAILED ---");
    }
    
    rclcpp::shutdown();
    spinner.join();
    return 0;
}