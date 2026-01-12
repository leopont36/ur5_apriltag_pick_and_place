#ifndef SWAP_COORDINATOR_HPP_
#define SWAP_COORDINATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "group18_assignment_2/action/move_to_pose.hpp"
#include "group18_assignment_2/action/gripper.hpp"
#include "group18_assignment_2/srv/gripper_request.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

class SwapCoordinator : public rclcpp::Node {
public:
    using MoveToPose = group18_assignment_2::action::MoveToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<MoveToPose>;

    using GripperAction = group18_assignment_2::action::Gripper;
    using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperAction>;
    
    SwapCoordinator();
    bool swapTags(const std::string& tag1_frame, const std::string& tag2_frame);
    
private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp_action::Client<MoveToPose>::SharedPtr action_client_;
    // rclcpp::Client<group18_assignment_2::srv::GripperRequest>::SharedPtr gripper_client_;
    rclcpp_action::Client<group18_assignment_2::action::Gripper>::SharedPtr gripper_action_client_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    geometry_msgs::msg::PoseStamped computeApproachPose(const geometry_msgs::msg::PoseStamped& target, double dist);

    bool getGraspPose(const std::string& tag_frame, geometry_msgs::msg::PoseStamped& grasp_pose);
    
    bool moveArmOverTarget(geometry_msgs::msg::PoseStamped pose, bool orientation);
    
    bool controlGripper(const std::string& cmd, const std::string& object_id = "");
    bool pickAndPlace(geometry_msgs::msg::PoseStamped pick, geometry_msgs::msg::PoseStamped place, const std::string& object_id);
};

#endif  // SWAP_COORDINATOR_HPP_