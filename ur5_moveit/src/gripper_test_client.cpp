// TO DELETE 
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <string>

#include "ur5_moveit/srv/gripper_request.hpp"

using namespace std::chrono_literals;

class GripperTestClient : public rclcpp::Node
{
public:
    GripperTestClient() : Node("gripper_test_client")
    {
        // create the client
        client_ = this->create_client<ur5_moveit::srv::GripperRequest>("gripper_service");
    }

    void send_command(std::string cmd)
    {
        // wait for the service to be available
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service 'gripper_service' not available, waiting again...");
        }

        // create and send the request
        auto request = std::make_shared<ur5_moveit::srv::GripperRequest::Request>();
        request->command = cmd;
        RCLCPP_INFO(this->get_logger(), "Sending request: '%s'", cmd.c_str());
        auto result_future = client_->async_send_request(request);

        // wait result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result_future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "SUCCESS! Message: %s", response->message.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "FAILED. Message: %s", response->message.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service gripper_service");
        }
    }

private:
    rclcpp::Client<ur5_moveit::srv::GripperRequest>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperTestClient>();
    node->send_command("open");
    rclcpp::shutdown();
    return 0;
}