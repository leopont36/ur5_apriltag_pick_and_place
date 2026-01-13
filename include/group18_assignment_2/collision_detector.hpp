#ifndef GROUP18_ASSIGNMENT_2__COLLISION_DETECTOR_HPP_
#define GROUP18_ASSIGNMENT_2__COLLISION_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>

class CollisionDetector : public rclcpp::Node
{
public:
    CollisionDetector();

private:
    void addCollisionTables();
};

#endif