#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include <chrono>

using namespace std::chrono_literals;

class TemplateDroneControl : public rclcpp::Node, public std::enable_shared_from_this<TemplateDroneControl>
{
public:
    TemplateDroneControl() : Node("template_drone_control_node")
    {
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);

        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

        // Adjust QoS settings for the subscription
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

        local_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/mavros/local_position/pose", qos, std::bind(&TemplateDroneControl::local_pos_cb, this, std::placeholders::_1));

    }

    void initialize()
    {
        std::cout << "Drone Control Initialized." << std::endl;
        set_guided_mode();
        arm_drone();
        takeoff(0.25);
    }

private:
    void set_guided_mode()
    {
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "GUIDED";

        while (!set_mode_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for set_mode service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
        }

        auto result = set_mode_client_->async_send_request(request);
        rclcpp::spin_until_future_complete(std::enable_shared_from_this<TemplateDroneControl>::shared_from_this(), result);
        RCLCPP_INFO(this->get_logger(), "GUIDED mode set.");
    }

    void arm_drone()
    {
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;

        while (!arming_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for arming service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for arming service...");
        }

        auto result = arming_client_->async_send_request(request);
        rclcpp::spin_until_future_complete(std::enable_shared_from_this<TemplateDroneControl>::shared_from_this(), result);
        RCLCPP_INFO(this->get_logger(), "Drone armed.");
    }

    void takeoff(double altitude)
    {
        auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        request->altitude = altitude;

        while (!takeoff_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for takeoff service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for takeoff service...");
        }

        auto result = takeoff_client_->async_send_request(request);
        rclcpp::spin_until_future_complete(std::enable_shared_from_this<TemplateDroneControl>::shared_from_this(), result);
        RCLCPP_INFO(this->get_logger(), "Takeoff initiated.");
    }

    void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped current_local_pos_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Current Local Position: %f, %f, %f", 
                    current_local_pos_.pose.position.x, 
                    current_local_pos_.pose.position.y, 
                    current_local_pos_.pose.position.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;
    mavros_msgs::msg::State current_state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TemplateDroneControl>();
    node->initialize(); // Call initialize after the object is fully managed by a shared_ptr
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
