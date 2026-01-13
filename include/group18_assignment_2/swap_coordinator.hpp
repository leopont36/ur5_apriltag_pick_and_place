#ifndef SWAP_COORDINATOR_HPP_
#define SWAP_COORDINATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "group18_assignment_2/action/move_to_pose.hpp"
#include "group18_assignment_2/action/gripper.hpp"
#include "group18_assignment_2/srv/gripper_request.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp" 
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <thread>
#include <cmath> 

/**
 * @brief Central supervisor node responsible for orchestrating the pick-and-place task.
 * This class implements the high-level logic of the application, acting as a client
 * for both the motion planner and the gripper controller. It synchronizes the 
 * perception data coming from the TF tree with the actuation subsystem, ensuring 
 * the correct execution of the three-phase swapping sequence.
 */
class SwapCoordinator : public rclcpp::Node {
public:
    using MoveToPose = group18_assignment_2::action::MoveToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<MoveToPose>;

    using GripperAction = group18_assignment_2::action::Gripper;
    using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperAction>;
    
    /**
     * @brief Constructor for the SwapCoordinator node.
     * Initializes the TF listeners for perception and establishes the action client 
     * connections with the lower-level controllers.
     */
    SwapCoordinator();

    /**
     * @brief Main routine executing the swapping logic between two AprilTags.
     * This method manages the high-level state machine, coordinating the transfer
     * of the first object to a temporary buffer and the subsequent placement of 
     * both objects to their final destinations.
     * @param tag1_frame The TF frame name of the first object.
     * @param tag2_frame The TF frame name of the second object.
     * @return true if the entire sequence is completed successfully.
     */
    bool swapTags(const std::string& tag1_frame, const std::string& tag2_frame);
    
private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp_action::Client<MoveToPose>::SharedPtr action_client_;
    rclcpp_action::Client<group18_assignment_2::action::Gripper>::SharedPtr gripper_action_client_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    /**
     * @brief Computes a safety position aligned with the gripper's z-axis.
     * Calculates a strictly linear offset from the target pose to determine the 
     * approach and retreat waypoints, ensuring the gripper enters and leaves the 
     * grasping area without lateral collisions.
     * @param target The grasping pose.
     * @param dist The distance in meters to back away from the target.
     * @return The computed approach pose.
     */
    geometry_msgs::msg::PoseStamped computeApproachPose(const geometry_msgs::msg::PoseStamped& target, double dist);

    /**
     * @brief queries the TF tree to calculate the optimal grasping pose.
     * Transforms the detected tag position from the camera frame to the robot base frame
     * and applies the necessary static offsets to align the end-effector center 
     * with the object centroid.
     * @param tag_frame The target TF frame.
     * @param grasp_pose Output parameter to store the calculated pose.
     * @return true if the transform is valid and available.
     */
    bool getGraspPose(const std::string& tag_frame, geometry_msgs::msg::PoseStamped& grasp_pose);

    /**
     * @brief Wrapper for the MotionPlanner action client.
     * Sends an asynchronous goal to the motion planner and waits for the result,
     * handling potential timeouts or execution failures.
     * @param pose The Cartesian target pose.
     * @param orientation Flag to enable orientation constraints during transport.
     * @param message Optional control message (e.g., "FINISH").
     */
    bool moveArmOverTarget(geometry_msgs::msg::PoseStamped pose, bool orientation, std::string message = "");

    /**
     * @brief Wrapper for the GripperNode action client.
     * Sends commands (open, close, grasp) to the end-effector controller.
     */
    bool controlGripper(const std::string& cmd);

    /**
     * @brief Executes the atomic pick-and-place sequence.
     * Implements the robust sequence of approach, grasp (with lift-off), transport 
     * (with constraints), and placement. This method encapsulates the geometric 
     * logic required to move an object safely from point A to point B.
     */
    bool pickAndPlace(geometry_msgs::msg::PoseStamped pick, geometry_msgs::msg::PoseStamped place);
};

#endif  // SWAP_COORDINATOR_HPP_