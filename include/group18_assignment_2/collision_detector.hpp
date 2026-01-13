#ifndef GROUP18_ASSIGNMENT_2__COLLISION_DETECTOR_HPP_
#define GROUP18_ASSIGNMENT_2__COLLISION_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <thread>

/**
 * @brief Node responsible for populating the MoveIt Planning Scene with static obstacles.
 * * Since the motion planner operates blindly regarding the Gazebo environment, this class
 * explicitly defines the "forbidden zones" (tables) to prevent physical collisions 
 * between the robotic arm and the environment during trajectory execution.
 */
class CollisionDetector : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the CollisionDetector node.
     * * Initializes the node and triggers the insertion of collision objects 
     * after a brief delay to ensure the MoveIt interface is ready.
     */
    CollisionDetector();

private:
    /**
     * @brief Defines and injects the geometric primitives into the collision matrix.
     * * This method creates two box primitives representing the cafe tables 
     * at coordinates derived from the Gazebo world. It uses the PlanningSceneInterface
     * to apply these objects, ensuring the OMPL solver rejects any path intersecting them.
     */
    void addCollisionTables();
};

#endif // GROUP18_ASSIGNMENT_2__COLLISION_DETECTOR_HPP_