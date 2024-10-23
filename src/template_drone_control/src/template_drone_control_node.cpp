    #include <rclcpp/rclcpp.hpp>
    #include <geometry_msgs/msg/pose_stamped.hpp>
    #include <mavros_msgs/msg/state.hpp>
    #include <mavros_msgs/srv/command_bool.hpp>
    #include <mavros_msgs/srv/set_mode.hpp>
    #include <mavros_msgs/srv/command_tol.hpp>

    #include "pgm_map_loader.hpp"
    #include "drone_navigation.hpp"

    using namespace std::chrono_literals;

    class TemplateDroneControl : public rclcpp::Node
    {
    public:
        TemplateDroneControl() : Node("template_drone_control_node")
        {
            PGMMapLoader map_loader;
            std::vector<std::string> map_names = map_loader.generateMapFilenames();
            for (auto &map_name : map_names)
            {
                RCLCPP_INFO(this->get_logger(), "Loading map: %s", map_name.c_str());
                // Use the map data
            }
            
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

            set_mode("GUIDED");
            set_arm();
            std::this_thread::sleep_for(500ms);
            // change_altitude(0.75);
            
            geometry_msgs::msg::Pose goal_pose;
            goal_pose.position.x = 0.0;  // Set the x-coordinate of your waypoint
            goal_pose.position.y = 1.5;  // Set the y-coordinate of your waypoint
            goal_pose.position.z = 0.75; // Desired altitude

            // Get drone's current position
            geometry_msgs::msg::Pose drone_position;
            drone_position.position = current_local_pos_.pose.position;

            // // Use your path generator
            map_loader.loadMap(map_names[2]);
            nav_msgs::msg::OccupancyGrid map = map_loader.getOccupancyGrid();
            RCLCPP_INFO(this->get_logger(), "Current Local Position: %f, %f, %f",
                        current_local_pos_.pose.position.x, current_local_pos_.pose.position.y, current_local_pos_.pose.position.z);
            nav_msgs::msg::Path path = generatePath(map, drone_position, goal_pose);
            // RCLCPP_INFO(this->get_logger(), "Current Local Position: %f, %f, %f",
            //             current_local_pos_.pose.position.x, current_local_pos_.pose.position.y, current_local_pos_.pose.position.z);
            for(auto pose_stamped : path.poses){
                RCLCPP_INFO(this->get_logger(), "Path: %f, %f, %f", pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z);
            }
            // set_arm();
            go_to_point((-1.0)*(path.poses.back().pose.position.x),(-1.0)*(path.poses.back().pose.position.y), 0.75);

        }   

    private:
        geometry_msgs::msg::PoseStamped current_local_pos_;
        void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            current_local_pos_ = *msg;
            // Get and log the current local position of the drone
            RCLCPP_INFO(this->get_logger(), "Current Local Position: %f, %f, %f",
                        current_local_pos_.pose.position.x, current_local_pos_.pose.position.y, current_local_pos_.pose.position.z);
        }

        void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
        {
            current_state_ = *msg;
            // RCLCPP_INFO(this->get_logger(), "Current State: %s", current_state_.mode.c_str());
        }
        void set_arm(){
            // Arm the drone
            auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_request->value = true;

            while (!arming_client_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the set_mode service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
            }

            auto arm_result = arming_client_->async_send_request(arm_request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), arm_result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                if (arm_result.get()->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Drone armed successfully.");
                    
                    // Wait for acknowledgment
                    while (!current_state_.armed)
                    {
                        rclcpp::spin_some(this->get_node_base_interface());
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to arm the drone.");
                    return;
                }
            }
        }
        void set_mode(char *mode){
            // Set mode to GUIDED
            mavros_msgs::srv::SetMode::Request guided_set_mode_req;
            guided_set_mode_req.custom_mode = mode;
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
                RCLCPP_INFO(this->get_logger(), "Drone mode changed to %s", mode);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to change mode to %s", mode);
                return;
            }
        }
        void go_to_point(double x, double y, double z){
            // Go to a specific point
            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.pose.position.x = x; // Set target position
            target_pose.pose.position.y = y;
            target_pose.pose.position.z = z; // Altitude
            RCLCPP_INFO(this->get_logger(), "Sending position command: x=%f, y=%f, z=%f", 
                        target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
            rclcpp::Rate rate(10); // Set the frequency to 10 Hz
            while (rclcpp::ok())
            {
                local_pos_pub_->publish(target_pose);
                if ((current_local_pos_.pose.position.x * 0.95 > current_local_pos_.pose.position.x) && (current_local_pos_.pose.position.x * 1.05 < current_local_pos_.pose.position.x) && 
                     (current_local_pos_.pose.position.y * 0.95 > current_local_pos_.pose.position.y) && (current_local_pos_.pose.position.y * 1.05 < current_local_pos_.pose.position.y)) // Implement this condition based on your use case
                {
                    RCLCPP_INFO(this->get_logger(), "Arrived at target position.");
                    break;
                }
            }
        }
        void change_altitude(double altitude){
            // Arm the drone
            auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            arm_request->value = true;
            auto arm_result = arming_client_->async_send_request(arm_request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), arm_result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                if (arm_result.get()->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Drone armed successfully.");
                    
                    // Wait for acknowledgment
                    while (!current_state_.armed)
                    {
                        rclcpp::spin_some(this->get_node_base_interface());
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }

                    // Takeoff command
                    auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
                    takeoff_request->altitude = altitude;  // Set desired altitude for takeoff
                    auto takeoff_result = takeoff_client_->async_send_request(takeoff_request);
                    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), takeoff_result) == rclcpp::FutureReturnCode::SUCCESS)
                    {
                        if (takeoff_result.get()->success)
                        {
                            RCLCPP_INFO(this->get_logger(), "Takeoff command sent successfully, awaiting acknowledgment...");
                            // Wait for takeoff acknowledgment
                            while (true)
                            {
                                if ((current_local_pos_.pose.position.z * 0.95 > current_local_pos_.pose.position.z) && (current_local_pos_.pose.position.z * 1.05 < current_local_pos_.pose.position.z) ) // Implement this condition based on your use case
                                {
                                    RCLCPP_INFO(this->get_logger(), "Takeoff acknowledged, drone is airborne.");
                                    break;
                                }
                                rclcpp::spin_some(this->get_node_base_interface());
                                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                            }
                        }
                        else
                        {
                            RCLCPP_ERROR(this->get_logger(), "Failed to send takeoff command.");
                            return;
                        }
                    }
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to arm the drone.");
                    return;
                }
            }
            // Takeoff command
            std::cout << "Takeoff acknowledged, drone is airborne." << std::endl;
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
