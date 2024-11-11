    #include <rclcpp/rclcpp.hpp>
    #include <geometry_msgs/msg/pose_stamped.hpp>
    #include <mavros_msgs/msg/state.hpp>
    #include <mavros_msgs/srv/command_bool.hpp>
    #include <mavros_msgs/srv/set_mode.hpp>
    #include <mavros_msgs/srv/command_tol.hpp>
    #include <chrono>
    #include <thread>
    #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
    #include "tf2/utils.h"

    using namespace std::chrono_literals;
    #include <iostream> 

    #include "pgm_map_loader.hpp"
    #include "drone_navigation.hpp"


    using namespace std::chrono_literals;

    struct map_index{
        double height;
        int index;
    };
    class TemplateDroneControl : public rclcpp::Node
    {
    public:
        TemplateDroneControl() : Node("template_drone_control_node")
        {
            std::vector<map_index> map_heights;
            map_heights.push_back({0.25, 0});
            map_heights.push_back({0.75, 1});
            map_heights.push_back({0.8, 2});
            map_heights.push_back({1.0, 3});
            map_heights.push_back({1.25, 4});
            map_heights.push_back({1.5, 5});
            map_heights.push_back({1.75, 6});
            map_heights.push_back({1.8, 7});
            map_heights.push_back({2.0, 8});
            map_heights.push_back({2.25, 9});

            PGMMapLoader map_loader;
            std::vector<std::string> map_names = map_loader.generateMapFilenames();
            for (auto &map_name : map_names)
            {
                RCLCPP_INFO(this->get_logger(), "Loading map: %s", map_name.c_str());
                // Use the map data
            }

            double start_x = current_local_pos_.pose.position.x;
            double start_y = current_local_pos_.pose.position.y;
            std::vector<Waypoint> waypoints = map_loader.loadWaypoints("src/LRS-FEI/mission_1_all.csv");
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

            double precision = 0.1;

            std::string key = "yaw";
            size_t pos;
            std::string numberStr;
            double curent_z;
            for (auto &waypoint : waypoints)
            {
                geometry_msgs::msg::Pose goal_pose;
                goal_pose.position.x = waypoint.x ;  // Set the x-coordinate of your waypoint
                goal_pose.position.y = (288*0.05)-waypoint.y ;  // Set the y-coordinate of your waypoint
                RCLCPP_INFO(this->get_logger(), "Waypoint: %f, %f", goal_pose.position.x, goal_pose.position.y);
                RCLCPP_INFO(this->get_logger(), "Waypoint z: %f", waypoint.z);
                goal_pose.position.z = waypoint.z; // Desired altitude
                std::cout << "Waypoint: " << waypoint.task << std::endl;
                pos = waypoint.task.find(key);
                if (pos != std::string::npos) {
                    // "yaw" is found; parse the number after it
                    numberStr = waypoint.task.substr(pos + key.length());
                }
                if(waypoint.task == "takeoff"){
                    takeoff(waypoint.z);
                    RCLCPP_INFO(this->get_logger(), "Taking off...");
                }else if(waypoint.task == "land"){
                }else if(waypoint.task == "landtakeoff"){
                }else{
                    change_altitude(goal_pose.position.z);
                    std::cout << "Waypoint: " << waypoint.task << "\n\n\n\n";
                }
                if (waypoint.precision == "hard") {
                    precision = 0.05;
                } else if (waypoint.precision == "soft") {
                    precision = 0.1;
                }

                
                // Get drone's current position
                geometry_msgs::msg::Pose drone_position;

                drone_position.position.x = current_local_pos_.pose.position.y + 13.6;
                drone_position.position.y = (288*0.05)+current_local_pos_.pose.position.x - 1.5;
                // Use your path generator
                for(int i = 0; i < 10; i++){
                    if(map_heights[i].height >= goal_pose.position.z){
                        std::cout << "Map index: " << map_heights[i].index << std::endl;
                        std::cout << map_names[map_heights[i].index] << std::endl;
                        map_loader.loadMap(map_names[map_heights[i].index]);
                        break;
                    }
                }
                nav_msgs::msg::OccupancyGrid map = map_loader.getOccupancyGrid();
                RCLCPP_INFO(this->get_logger(), "Current Local Position: %f, %f, %f",
                            current_local_pos_.pose.position.x, current_local_pos_.pose.position.y, current_local_pos_.pose.position.z);
                nav_msgs::msg::Path path = generatePath(map, drone_position, goal_pose);

                for(auto pose_stamped : path.poses){
                    pose_stamped.pose.position.x = pose_stamped.pose.position.x;
                    pose_stamped.pose.position.y = (288*0.05) - pose_stamped.pose.position.y;
                    RCLCPP_INFO(this->get_logger(), "Next point: %f, %f, %f", pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z);
                    if (&pose_stamped == &path.poses.back()) {
                        go_to_point((-1.0)*(pose_stamped.pose.position.y-1.5), (pose_stamped.pose.position.x-13.6), waypoint.z, precision);
                    } else {
                        go_to_point((-1.0)*(pose_stamped.pose.position.y-1.5), (pose_stamped.pose.position.x-13.6), waypoint.z, 0.25);
                    }
                }
                // TODO change land takeoff to set_mode("LAND") and set_mode("GUIDED")
                if(waypoint.task == "landtakeoff"){
                    curent_z = current_local_pos_.pose.position.z;
                    // change_altitude(0);
                    set_mode("LAND");
                    set_mode("GUIDED"); 
                    takeoff(curent_z);
                    RCLCPP_INFO(this->get_logger(), "landtakeoff...");
                }else if(waypoint.task == "land"){
                    // change_altitude(0);
                    set_mode("LAND");
                    RCLCPP_INFO(this->get_logger(), "Landing...");
                }                
                try {
                    int number = std::stoi(numberStr); // Convert to integer
                    std::cout << "Found yaw with value: " << number << std::endl;   
                    orient(number);
                    numberStr = "";
                } catch (std::invalid_argument&) {
                    std::cerr << "No valid number found after yaw" << std::endl;
                }
                
            }
        }   

    private:
    double normalizeAngle(double angle) {
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;
            return angle;
        }
        geometry_msgs::msg::PoseStamped current_local_pos_;
        void local_pos_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            current_local_pos_ = *msg;
            // Get and log the current local position of the drone
            // RCLCPP_INFO(this->get_logger(), "Current Local Position: %f, %f, %f",
            //             current_local_pos_.pose.position.x, current_local_pos_.pose.position.y, current_local_pos_.pose.position.z);
            // RCLCPP_INFO(this->get_logger(), "Current Local Orientation: %f, %f, %f, %f",
            //             current_local_pos_.pose.orientation.x, current_local_pos_.pose.orientation.y, current_local_pos_.pose.orientation.z, current_local_pos_.pose.orientation.w);
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
            if(mode == "LAND"){
                while(current_local_pos_.pose.position.z > 0.15){
                    rclcpp::spin_some(this->get_node_base_interface());
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
        }
        void go_to_point(double x, double y, double z, double tolerance){
            // Go to a specific point
            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.pose.position.x = x; // Set target position
            target_pose.pose.position.y = y;
            target_pose.pose.position.z = z; // Maintain current altitude
            target_pose.pose.orientation.x = current_local_pos_.pose.orientation.x;
            target_pose.pose.orientation.y = current_local_pos_.pose.orientation.y;
            target_pose.pose.orientation.z = current_local_pos_.pose.orientation.z;
            target_pose.pose.orientation.w = current_local_pos_.pose.orientation.w;

            RCLCPP_INFO(this->get_logger(), "Sending position command: x=%f, y=%f, z=%f", 
                       target_pose.pose.position.x*(-1.0)+1.5, target_pose.pose.position.y+13.6, target_pose.pose.position.z);
            rclcpp::Rate rate(10); // Set the frequency to 10 Hz
            while (rclcpp::ok())
            {
                local_pos_pub_->publish(target_pose);
                
                 
                if (std::abs(current_local_pos_.pose.position.x - target_pose.pose.position.x) <= tolerance &&
                    std::abs(current_local_pos_.pose.position.y - target_pose.pose.position.y) <= tolerance &&
                    std::abs(current_local_pos_.pose.position.z - target_pose.pose.position.z) <= tolerance) {
                    RCLCPP_INFO(this->get_logger(), "Arrived at target position.");
                    break;
                }

                rclcpp::spin_some(this->get_node_base_interface());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        
        void orient(double yaw) {
            rclcpp::Rate rate(10); // Set frequency to 10 Hz

            // Get the current yaw orientation in radians
            double current_yaw = normalizeAngle(tf2::getYaw(current_local_pos_.pose.orientation));
            double target_yaw = normalizeAngle(current_yaw + yaw * M_PI / 180.0);

            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.pose.position = current_local_pos_.pose.position;

            while (rclcpp::ok()) {
                // Update the target orientation in the loop
                tf2::Quaternion quaternion;
                quaternion.setRPY(0.0, 0.0, target_yaw); // Roll, Pitch, Yaw (Z-axis rotation)
                target_pose.pose.orientation = tf2::toMsg(quaternion);

                // Publish the orientation command
                local_pos_pub_->publish(target_pose);

                // Get current yaw after publishing
                double current_yaw_updated = tf2::getYaw(current_local_pos_.pose.orientation);

                // Log current and target yaw
                // RCLCPP_INFO(this->get_logger(), "Current yaw: %f, Target yaw: %f",
                            // current_yaw_updated, target_yaw);

                // Check if within tolerance (0.05 radians ~ 2.9 degrees)
                if (std::abs(current_yaw_updated - target_yaw) <= 0.15) {
                    RCLCPP_INFO(this->get_logger(), "Arrived at target orientation.");
                    break;
                }

                rclcpp::spin_some(this->get_node_base_interface());
                rate.sleep();
            }
        }
        void takeoff(double altitude){
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
                                const double tolerance = 0.025; // 5 cm tolerance
                                    if (std::abs(current_local_pos_.pose.position.z - altitude) <= tolerance)
                                    {
                                        RCLCPP_INFO(this->get_logger(), "Takeoff acknowledged, drone is at desired height.");
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
        }
        void change_altitude(double altitude){
            // Go to a specific point
            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.pose.position.x = current_local_pos_.pose.position.x; // Set target position
            target_pose.pose.position.y = current_local_pos_.pose.position.y;
            target_pose.pose.position.z = altitude; // Maintain current altitude
            target_pose.pose.orientation.x = current_local_pos_.pose.orientation.x;
            target_pose.pose.orientation.y = current_local_pos_.pose.orientation.y;
            target_pose.pose.orientation.z = current_local_pos_.pose.orientation.z;
            target_pose.pose.orientation.w = current_local_pos_.pose.orientation.w;
            RCLCPP_INFO(this->get_logger(), "Sending altitude command: z=%f", 
                target_pose.pose.position.z);
            rclcpp::Rate rate(10); // Set the frequency to 10 Hz
            while (rclcpp::ok())
            {
                local_pos_pub_->publish(target_pose);
                 
                if (
                    std::abs(current_local_pos_.pose.position.z - target_pose.pose.position.z) <= 0.2) {
                    RCLCPP_INFO(this->get_logger(), "Arrived at target position.");
                    break;
                }

                rclcpp::spin_some(this->get_node_base_interface());
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
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
