#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include <memory>

using CommandBool = mavros_msgs::srv::CommandBool;

class MissionTaskServer : public rclcpp::Node
{
public:
    MissionTaskServer() : Node("mission_task_server")
    {
        stop_service_ = this->create_service<CommandBool>(
            "/drone/cmd/stop",
            std::bind(&MissionTaskServer::handle_stop_service, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Stop service ready at /drone/cmd/stop");
    }

private:
    void handle_stop_service(
        const std::shared_ptr<CommandBool::Request> request,
        std::shared_ptr<CommandBool::Response> response)
    {
        if (request->value)
        {
            RCLCPP_INFO(this->get_logger(), "Received Stop command: value = true");
            // Add your logic to stop the UAV
            response->success = true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Stop command ignored: value = false");
            response->success = false;
        }
    }

    rclcpp::Service<CommandBool>::SharedPtr stop_service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionTaskServer>());
    rclcpp::shutdown();
    return 0;
}
