#ifndef GROUP18_ASSIGNMENT_2__COLLISION_DETECTOR_HPP_
#define GROUP18_ASSIGNMENT_2__COLLISION_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <thread>

class CollisionDetector : public rclcpp::Node
{
public:
    CollisionDetector();

private:
    void addCollisionTables();
};

#endif