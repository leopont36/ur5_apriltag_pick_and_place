#include "hello_moveit/collision_detector.hpp"


CollisionDetector::CollisionDetector(const rclcpp::NodeOptions & options) : Node("collision_detector")
{
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        std::chrono::seconds(2);

        RCLCPP_INFO(this->get_logger(), "Nodo CollisionDetector avviato.");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&CollisionDetector::addCollisionBox, this)
        );



}
    
void CollisionDetector::addCollisionBox()
{

        try {

            auto frame_cube_1 = tf_buffer_->lookupTransform(
            FRAME_ID,
            CUBE_1,
            tf2::TimePointZero
            );


            auto frame_cube_2 = tf_buffer_->lookupTransform(
            FRAME_ID,
            CUBE_2,
            tf2::TimePointZero 
            );

            RCLCPP_INFO(this->get_logger(), 
                "\n--- INFO TRASFORMAZIONE Trovata ---\n"
                "Frame Padre: %s -> Frame Figlio: %s\n"
                "Traslazione (metri): [X: %f, Y: %f, Z: %f]\n"
                "Rotazione (quat):    [X: %.3f, Y: %.3f, Z: %.3f, W: %.3f]",
                frame_cube_2.header.frame_id.c_str(),
                frame_cube_2.child_frame_id.c_str(),
                frame_cube_1.transform.translation.x,
                frame_cube_1.transform.translation.y,
                frame_cube_1.transform.translation.z,
                frame_cube_2.transform.rotation.x,
                frame_cube_2.transform.rotation.y,
                frame_cube_2.transform.rotation.z,
                frame_cube_2.transform.rotation.w
            );


            shape_msgs::msg::SolidPrimitive primitive;
            double width = 0.06 ;
            double height = 0.1;
            double depth = 0.06;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.BOX_X] = width;
            primitive.dimensions[primitive.BOX_Y] = depth;
            primitive.dimensions[primitive.BOX_Z] = height;

            moveit_msgs::msg::CollisionObject collision_cube_1;
            collision_cube_1.header.frame_id = FRAME_ID;
            collision_cube_1.id = CUBE_1;

            geometry_msgs::msg::Pose pose_cube_1;
            pose_cube_1.orientation.w = 1.0;
            pose_cube_1.position.x = frame_cube_1.transform.translation.x;
            pose_cube_1.position.y = frame_cube_1.transform.translation.y;//frame_cube_1.transform.translation.y;
            pose_cube_1.position.z = frame_cube_1.transform.translation.z -(height/2);//frame_cube_1.transform.rotation.z - 0.05;

            collision_cube_1.primitives.push_back(primitive);
            collision_cube_1.primitive_poses.push_back(pose_cube_1);
            collision_cube_1.operation = collision_cube_1.ADD;
            
            planning_scene_interface.applyCollisionObject(collision_cube_1);


            moveit_msgs::msg::CollisionObject collision_cube_2;
            collision_cube_2.header.frame_id = FRAME_ID;
            collision_cube_2.id = CUBE_2;

            geometry_msgs::msg::Pose pose_cube_2;
            pose_cube_2.orientation.w = 1.0;
            pose_cube_2.position.x = frame_cube_2.transform.translation.x;
            pose_cube_2.position.y = frame_cube_2.transform.translation.y;
            pose_cube_2.position.z = frame_cube_2.transform.translation.z -(height/2);

            collision_cube_2.primitives.push_back(primitive);
            collision_cube_2.primitive_poses.push_back(pose_cube_2);
            collision_cube_2.operation = collision_cube_2.ADD;

            planning_scene_interface.applyCollisionObject(collision_cube_2);


        }
        catch (const tf2::TransformException & ex)
        {
            RCLCPP_WARN(this->get_logger(), "Impossibile trovare la trasformazione: %s", ex.what());
            return;
        }

}




int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.parameter_overrides({
        {"use_sim_time", true}
    });

    auto node = std::make_shared<CollisionDetector>(node_options);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
