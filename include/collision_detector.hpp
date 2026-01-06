#ifndef COLLISION_DETECTOR_HPP_
#define COLLISION_DETECTOR_HPP_

// 1. Core ROS 2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>

// 2. Standard C++
#include <memory>
#include <string>
#include <chrono>

// 3. TF2 Headers
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// 4. MoveIt Headers
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>



class CollisionDetector : public rclcpp::Node
{
    public:

        CollisionDetector();
    
    private:

        void addCollisionBox();

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::TimerBase::SharedPtr timer_;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        const std::string FRAME_ID = "base_link";
        const std::string CUBE_1 = "tag36h11:1";
        const std::string CUBE_2 = "tag36h11:10";
};

#endif