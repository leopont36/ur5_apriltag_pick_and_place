#include "group18_assignment_2/color_detector.hpp"


ColorDetectorServer::ColorDetectorServer() : Node("colorDetectorServer")
{
        RCLCPP_INFO(this->get_logger(), "Starting Color Detector Node");

        //Service Group
        service_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        camera_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        //Options subscription
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = camera_group_;

        //Create the subscription to the camera topic
        camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
          "/rgb_camera/image", 
          rclcpp::SensorDataQoS(), 
          std::bind(&ColorDetectorServer::image_callback, this, std::placeholders::_1),
          sub_options
        );

        camera_info_subcription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
          "/rgb_camera/camera_info", 
          rclcpp::SensorDataQoS(),
          std::bind(&ColorDetectorServer::camera_info_callback, this, std::placeholders::_1),
          sub_options
        );


        //Create the tf2 subscription
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        
        //Create the service for the node
        service=this->create_service<detector_interfaces::srv::ColorDetection>("color_detect",
          [this](const std::shared_ptr<detector_interfaces::srv::ColorDetection::Request> request,
            std::shared_ptr<detector_interfaces::srv::ColorDetection::Response> response) {
              
            this->service_callback(request, response);
        },
        rclcpp::ServicesQoS(),
        service_group_
        );

}

    
void ColorDetectorServer::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
       
      rclcpp::sleep_for(std::chrono::seconds(2));
      std::lock_guard<std::mutex> lock(image_mutex_);
      last_msg_ = msg;
     
}

void ColorDetectorServer::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
        if(!has_cam_info)
        {
          camera_matrix_ = cv::Mat(3, 3, CV_64F);

          camera_matrix_.at<double>(0,0) = msg->k[0]; // fx
          camera_matrix_.at<double>(0,1) = msg->k[1]; // 0
          camera_matrix_.at<double>(0,2) = msg->k[2]; // cx

          camera_matrix_.at<double>(1,0) = msg->k[3]; // 0
          camera_matrix_.at<double>(1,1) = msg->k[4]; // fy
          camera_matrix_.at<double>(1,2) = msg->k[5]; // cy

          camera_matrix_.at<double>(2,0) = msg->k[6]; // 0
          camera_matrix_.at<double>(2,1) = msg->k[7]; // 0
          camera_matrix_.at<double>(2,2) = msg->k[8]; // 1

          dist_coeffs_ = msg->d;
          has_cam_info=true;
          
        }
        

}

cv::Point2d ColorDetectorServer::project_points(geometry_msgs::msg::TransformStamped frame,double off_x,double off_y,double off_z)
{
          tf2::Transform tf_cam_tag;
          tf2::fromMsg(frame.transform, tf_cam_tag);
          tf2::Vector3 point_in_tag(off_x, off_y, off_z);
          tf2::Vector3 point_in_cam = tf_cam_tag * point_in_tag;

          std::vector<cv::Point3d> object_points;
          object_points.push_back(cv::Point3d(point_in_cam.x(), point_in_cam.y(), point_in_cam.z()));

          std::vector<cv::Point2d> image_points;
          cv::projectPoints(object_points, cv::Vec3d(0,0,0), cv::Vec3d(0,0,0), camera_matrix_, dist_coeffs_, image_points);

          return image_points[0];
}

cv::Mat ColorDetectorServer::mask_point(cv::Mat& img,cv::Point2d center, int radius)
{
        int x_start = std::max(0, (int)center.x - radius);
        int y_start = std::max(0, (int)center.y - radius);
        int width = std::min(img.cols - x_start, radius * 2);
        int height = std::min(img.rows - y_start, radius * 2);

        cv::Rect roi_rect(x_start, y_start, width, height);  
        cv::Mat roi_mat = img(roi_rect); 

        return roi_mat;

}

std::string ColorDetectorServer::image_to_color(cv::Mat image)
{
        cv::Scalar mean_color = cv::mean(image);
        cv::Mat bgr_mat(1, 1, CV_8UC3, mean_color);  
        cv::Mat hsv_mat;
        cv::cvtColor(bgr_mat, hsv_mat, cv::COLOR_BGR2HSV);
        cv::Vec3b hsv_val = hsv_mat.at<cv::Vec3b>(0, 0);
        
        //Cheking only tonality to find the color
        int H = hsv_val[0];

        std::string detected_color;
        if (H < 10 || H > 170) {
            detected_color = "RED";
                 
        }
            // VERDE (Range: 35-85)
        else if (H >= 35 && H < 85) {
            detected_color = "GREEN";
                 
        }
            // BLU (Range: 90-135)
        else if (H >= 90 && H <= 135) {
            detected_color = "BLUE";
                
        }
        else {
                
            detected_color = "COLOR NOT FINDED";
        }

        return detected_color;

}
   

void ColorDetectorServer::service_callback(const std::shared_ptr<detector_interfaces::srv::ColorDetection::Request> request,
      std::shared_ptr<detector_interfaces::srv::ColorDetection::Response> response)
{

      RCLCPP_INFO(this->get_logger(), "Request Received");
      RCLCPP_INFO(this->get_logger(), "ID: '%s'", request->cube_id.c_str());
      bool isFinded=false;
      while(!isFinded)
      {
          try
        {
            rclcpp::sleep_for(std::chrono::seconds(5));
            auto frame_cube = tf_buffer_->lookupTransform(
                  FRAME_ID,
                  request->cube_id.c_str(),
                  tf2::TimePointZero
              );
              
            
            cv::Point2d pixel = project_points(frame_cube,0.0,0.0,-0.08);

            current_image_ = cv_bridge::toCvCopy(last_msg_, "bgr8")->image;
            
            cv::Mat masked = mask_point(current_image_,pixel,10);

            std::string detected_color=image_to_color(masked);

            RCLCPP_INFO(this->get_logger(),"Il colore è %s",detected_color.c_str());
            
            response->color=detected_color;   
            
            isFinded=true;

        }
        catch (const tf2::TransformException& ex)
        {
              RCLCPP_WARN(this->get_logger(), "Could not find transform: %s", ex.what());
             
        }
        catch(cv_bridge::Exception& e)
        {
            RCLCPP_WARN(this->get_logger(), "CV Bridge Error");
        }
      }
      
         
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ColorDetectorServer>();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
