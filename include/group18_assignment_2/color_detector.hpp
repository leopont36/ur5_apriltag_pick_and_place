#ifndef COLOR_DETECTOR_HPP_
#define COLOR_DETECTOR_HPP_

//Standard
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "group18_assignment_2/srv/color_detection.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

//TF2
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

//OpenCV
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>


/**
 * @brief This node is responsable to identify the color of the cube
 * given it's position from the AprilTag Node. This method work as 
 * a Service Server, providing in output the color of the requested cube
 * to the Client through a specific service request. The node utilizes 
 * distinct CallableGroup for the camera and the service, to ensure
 * concurrency and responsiveness.
 */
class ColorDetectorServer : public rclcpp::Node
{
    public:

        /**
        * @brief Constructor for the ColorDetectorServer class.
        *
        * Initializes the ROS 2 node "colorDetectorServer". It sets up mutually exclusive 
        * callback groups to handle concurrency, subscribes to camera topics 
        * (/rgb_camera/image and /rgb_camera/camera_info), initializes the TF2 buffer, 
        * and creates the "color_detect" service.
        */
        ColorDetectorServer();

    private:

        //Camera Subscription
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subcription_;

        //Service
        rclcpp::Service<group18_assignment_2::srv::ColorDetection>::SharedPtr service;

        //TF2
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        //Image
        cv::Mat current_image_;
        sensor_msgs::msg::Image::SharedPtr last_msg_;

        // CallBack group
        rclcpp::CallbackGroup::SharedPtr service_group_;
        rclcpp::CallbackGroup::SharedPtr camera_group_;
        std::mutex image_mutex_;

        //Camera Info
        bool has_cam_info = false;
        cv::Mat camera_matrix_;
        std::vector<double> dist_coeffs_;

        //Frame id
        const std::string FRAME_ID = "external_camera/link/rgb_camera";
        const std::string CUBE_1 = "tag36h11:1";
        const std::string CUBE_2 = "tag36h11:10";


        /**
        * @brief Callback for processing incoming frames
        *
        * This method is triggered whenever a new image message arrives.
        * It introduces a delay (sleep) of 2 seconds and stores the latest received 
        * message in a thread-safe member variable.
        *
        * @param msg Shared pointer to the image message (sensor_msgs::msg::Image).
        */
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);


        /**
        * @brief Callback for receiving camera intrinsic parameters.
        *
        * This method captures the CameraInfo (Matrix K and Distortion Coefficients D).
        * It runs only until the information is successfully acquired, 
        * after which it stops updating to save resources.
        *
        * @param msg Shared pointer to the camera info message (sensor_msgs::msg::CameraInfo).
        */
        void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);


        /**
        * @brief Projects a 3D point of the cube onto the 2D image plane of the camera.
        *
        * It uses the provided TF2 transform to calculate the object's position relative 
        * to the camera, then applies the camera's intrinsic matrix and distortion coefficients 
        * to determine the pixel coordinates (u, v).
        *
        * @param frame The TF2 transform describing the object's pose relative to the camera.
        * @param off_x X-axis offset from the target cube center.
        * @param off_y Y-axis offset from the target cube center.
        * @param off_z Z-axis offset from the target cube center (e.g., -0.08 to approach the front face).
        * @return cv::Point2d The coordinates (x, y) of the projected pixel on the image.
        */
        cv::Point2d project_points(geometry_msgs::msg::TransformStamped frame,double off_x,double off_y,double off_z);


        /**
        * @brief Extracts a Region of Interest from the image.
        *
        * Crops a rectangular portion of the source image centered around a specific point.
        * It includes boundary checks to ensure the region does not exceed the image dimensions.
        *
        * @param img The source image.
        * @param center The center pixel point for the extraction.
        * @param radius The "radius" (or half-width) of the area to crop.
        * @return cv::Mat A small image matrix containing only the region of interest.
        */
        cv::Mat mask_point(cv::Mat& img,cv::Point2d center, int radius);


        /**
        * @brief Identifies the color on a specific mask image received in input
        *
        * Calculates the mean color of the image, converts it from BGR to HSV color space,
        * and analyzes the Hue (H) channel to classify the color.
        *
        * Ranges used (approximate):
        * - RED:   H < 10 or H > 170
        * - GREEN: 35 <= H < 85
        * - BLUE:  90 <= H <= 135
        *
        * @param image The masked image to analyze.
        * @return std::string A string representing the detected color ("RED", "GREEN", "BLUE", or "COLOR NOT FINDED").
        */
        std::string image_to_color(cv::Mat image);


        /**
        * @brief Main callback for the color detection service.
        *
        * Orchestrates the detection pipeline:
        *  - Looks for the TF transform between the camera and the cube, with the id in input.
        *  - Projects the position of the cube in the image of the camera.
        *  - Crop the image and analyze the color in the ctopped area.
        * 7. Returns the result in the service response
        *
        * The loop continues until a valid transformation is found and the process completes.
        *
        * @param request service request (contains the cube ID).
        * @param response service response (to be filled with the detected color).
        */
        void service_callback(const std::shared_ptr<group18_assignment_2::srv::ColorDetection::Request> request,std::shared_ptr<group18_assignment_2::srv::ColorDetection::Response> response);


};

#endif // COLOR_DETECTOR_HPP_