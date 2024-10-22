#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>

using namespace std::chrono_literals;

class TemplateDroneControl : public rclcpp::Node
{
public:
    TemplateDroneControl() : Node("template_drone_control_node")
    {
        // Set up ROS publishers, subscribers, and service clients
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "mavros/state", 10, std::bind(&TemplateDroneControl::state_cb, this, std::placeholders::_1));
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
        takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

        // Create a QoS profile for receiving local position data
        rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
        custom_qos.depth = 1;
        custom_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(custom_qos.history, 1), custom_qos);
        local_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos, std::bind(&TemplateDroneControl::local_pos_cb, this, std::placeholders::_1));

        // Wait for MAVROS SITL connection
        while (rclcpp::ok() && !current_state_.connected)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(100ms);
        }

        // Set mode to GUIDED
        mavros_msgs::srv::SetMode::Request guided_set_mode_req;
        guided_set_mode_req.custom_mode = "GUIDED";
        while (!set_mode_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the set_mode service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
        }
        auto result = set_mode_client_->async_send_request(std::make_shared<mavros_msgs::srv::SetMode::Request>(guided_set_mode_req));
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Drone mode changed to GUIDED.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to change mode to GUIDED.");
            return;
        }
        
        // Arm the drone
        auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        arm_request->value = true;
        auto arm_result = arming_client_->async_send_request(arm_request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), arm_result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            if (arm_result.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), "Drone armed successfully.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to arm the drone.");
                return;
            }
        }

        // Takeoff
        auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        takeoff_request->altitude = 10.0;  // Set desired altitude for takeoff
        auto takeoff_result = takeoff_client_->async_send_request(takeoff_request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), takeoff_result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            if (takeoff_result.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), "Takeoff successful.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to take off.");
                return;
            }
        }

        // Implement position control: Sending a position command after takeoff
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.pose.position.x = 10.0; // Set target position
        target_pose.pose.position.y = 10.0;
        target_pose.pose.position.z = 10.0; // Altitude
        local_pos_pub_->publish(target_pose);

        RCLCPP_INFO(this->get_logger(), "Sending position command: x=%f, y=%f, z=%f", 
                    target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
    }

private:
    void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped current_local_pos_ = *msg;

        // Get and log the current local position of the drone
        RCLCPP_INFO(this->get_logger(), "Current Local Position: %f, %f, %f",
                    current_local_pos_.pose.position.x, current_local_pos_.pose.position.y, current_local_pos_.pose.position.z);
    }

    void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Current State: %s", current_state_.mode.c_str());
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;
    mavros_msgs::msg::State current_state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemplateDroneControl>());
    rclcpp::shutdown();
    return 0;
}
