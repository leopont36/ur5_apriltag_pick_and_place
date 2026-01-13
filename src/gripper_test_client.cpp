// TO DELETE
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <string>

#include "group18_assignment_2/srv/gripper_request.hpp"

using namespace std::chrono_literals;

class GripperTestClient : public rclcpp::Node
{
public:
    GripperTestClient() : Node("gripper_test_client")
    {
        client_ = this->create_client<group18_assignment_2::srv::GripperRequest>("gripper_service");
    }

    void send_command(std::string cmd)
    {
        // wait for service
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for 'gripper_service'...");
        }

        // build request
        auto request = std::make_shared<group18_assignment_2::srv::GripperRequest::Request>();
        request->command = cmd;

        RCLCPP_INFO(this->get_logger(), "Sending command: '%s'", cmd.c_str());
        
        // send async
        auto result_future = client_->async_send_request(request);

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
            RCLCPP_ERROR(this->get_logger(), "Service call failed or timed out.");
        }
    }

private:
    rclcpp::Client<group18_assignment_2::srv::GripperRequest>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    if (argc < 2) {
        printf("Usage: ros2 run group18_assignment_2 gripper_test_client [open|close]\n");
        rclcpp::shutdown();
        return 1;
    }

    std::string command = argv[1];

    auto node = std::make_shared<GripperTestClient>();
    node->send_command(command);

    rclcpp::shutdown();
    return 0;
}