#include "group18_assignment_2/collision_detector.hpp"
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <thread>

CollisionDetector::CollisionDetector() : Node("collision_detector")
{
    // Give MoveIt a moment to connect
    std::this_thread::sleep_for(std::chrono::seconds(2));
    addCollisionTables();
}

void CollisionDetector::addCollisionTables()
{
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;


    double table_w = 0.3; 
    double table_d = 0.3;
    double table_h = 0.3;

    shape_msgs::msg::SolidPrimitive prim;
    prim.type = prim.BOX;
    prim.dimensions = {table_w, table_d, table_h};

    // TABLE 1
    moveit_msgs::msg::CollisionObject t1;
    t1.header.frame_id = "world";
    t1.id = "cafe_table";
    t1.primitives.push_back(prim);

    geometry_msgs::msg::Pose p1;
    p1.orientation.w = 1.0;
    p1.position.x = 4.0;
    p1.position.y = -1.1;
    p1.position.z = table_h / 2.0;
    
    t1.primitive_poses.push_back(p1);
    t1.operation = t1.ADD;
    collision_objects.push_back(t1);

    // TABLE 2
    moveit_msgs::msg::CollisionObject t2;
    t2.header.frame_id = "world";
    t2.id = "cafe_table2";
    t2.primitives.push_back(prim);

    geometry_msgs::msg::Pose p2;
    p2.orientation.w = 1.0;
    p2.position.x = 4.6;  // Approx center based on Tag 10
    p2.position.y = -0.5; // Approx center based on Tag 10
    p2.position.z = table_h / 2.0; 

    t2.primitive_poses.push_back(p2);
    t2.operation = t2.ADD;
    collision_objects.push_back(t2);

    planning_scene_interface.applyCollisionObjects(collision_objects);
    RCLCPP_INFO(this->get_logger(), "Hardcoded collision objects added!");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CollisionDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}