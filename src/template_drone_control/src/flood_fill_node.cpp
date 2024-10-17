#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <vector>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

struct Waypoint {
    double x;
    double y;
    double z;
    double precision;
    std::string task;
};

class FloodFillDroneControl : public rclcpp::Node
{
public:
    FloodFillDroneControl() : Node("flood_fill_drone_control_node")
    {
        // Initialize subscriptions, publishers, and clients (e.g., MAVROS services)
        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("mavros/setpoint_position/local", 10);

        // Load the point cloud data from a PCD file
        load_pcd_file("src/LRS-FEI/maps/FEI_LRS_PCD/map.pcd");

        // Define waypoints with tasks
        waypoints_ = {
            {13.60, 1.50, 1.00, 0.5, "takeoff"},
            {8.65, 2.02, 1.00, 0.5, ""},
            {4.84, 5.37, 2.00, 0.5, "yaw180"},
            {2.08, 9.74, 1.75, 0.5, ""},
            {8.84, 6.90, 2.00, 0.5, "landtakeoff"},
            {2.81, 8.15, 1.50, 0.5, "yaw90"},
            {13.60, 1.50, 2.00, 0.5, "land"}
        };

        // Start the navigation process
        current_waypoint_ = 0;
        navigate_to_waypoint();
        
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    std::vector<Waypoint> waypoints_;
    std::vector<geometry_msgs::msg::Point> trajectory_; // To store the trajectory

    size_t current_waypoint_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;  // Point cloud data

    void load_pcd_file(const std::string &pcd_file) {
        cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud_) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", pcd_file.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "Loaded point cloud with %zu points", cloud_->size());
    }

    // Adjust the function to take in geometry_msgs::msg::Point instead of Waypoint
    bool check_for_collision(const geometry_msgs::msg::Point& start, const geometry_msgs::msg::Point& end) {
        double safety_margin = 0.5;  // Safety margin radius for collision checking

        for (const auto& point : cloud_->points) {
            double distance_to_path = point_to_line_distance(point, start, end);
            if (distance_to_path < safety_margin) {
                RCLCPP_WARN(this->get_logger(), "Collision detected at point (%f, %f, %f)", point.x, point.y, point.z);
                return true;
            }
        }
        return false;
    }

        // Modify reroute to use geometry_msgs::msg::Point
    void reroute(geometry_msgs::msg::Point& current_point, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        float delta = 0.5;  // step size to avoid obstacle
        geometry_msgs::msg::Point new_point = current_point;

        while (check_for_collision(new_point, current_point)) {
            new_point.x += delta;  // move in X direction
            if (!check_for_collision(new_point, current_point)) break;
            new_point.x -= delta;
            new_point.y += delta;  // move in Y direction
            if (!check_for_collision(new_point, current_point)) break;
        }   
        RCLCPP_INFO(this->get_logger(), "New point [%f, %f, %f]", new_point.x, new_point.y, new_point.z);
        current_point = new_point;  // update current point to rerouted position
    }


    // Modify the point_to_line_distance to also use geometry_msgs::msg::Point
    double point_to_line_distance(const pcl::PointXYZ &point, const geometry_msgs::msg::Point &start, const geometry_msgs::msg::Point &end) {
        Eigen::Vector3d p(point.x, point.y, point.z);
        Eigen::Vector3d s(start.x, start.y, start.z);
        Eigen::Vector3d e(end.x, end.y, end.z);

        Eigen::Vector3d d = e - s;
        Eigen::Vector3d ps = p - s;
        double t = ps.dot(d) / d.dot(d);
        t = std::clamp(t, 0.0, 1.0);

        Eigen::Vector3d closest_point = s + t * d;
        return (p - closest_point).norm();
    }

    void navigate_to_waypoint() {
        if (current_waypoint_ >= waypoints_.size()) {
            RCLCPP_INFO(this->get_logger(), "All waypoints reached.");
            print_trajectory();
            return;
        }

        const Waypoint& target = waypoints_[current_waypoint_];

        // Create a geometry_msgs::msg::Point for the current position
        geometry_msgs::msg::Point current_point;
        current_point.x = waypoints_[current_waypoint_ - 1].x;
        current_point.y = waypoints_[current_waypoint_ - 1].y;
        current_point.z = waypoints_[current_waypoint_ - 1].z;

        geometry_msgs::msg::Point target_point;
        target_point.x = waypoints_[current_waypoint_].x;
        target_point.y = waypoints_[current_waypoint_].y;
        target_point.z = waypoints_[current_waypoint_].z;
        // Check for collision between the current position and the target waypoint
        if (current_waypoint_ > 0 && check_for_collision(current_point, target_point)) {
            reroute(current_point, cloud_);
            trajectory_.push_back(current_point);
            RCLCPP_ERROR(this->get_logger(), "Collision detected in path, cannot proceed to waypoint %zu", current_waypoint_);
        }else{
            trajectory_.push_back(current_point);
        }

        // Publish target position
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = target.x;
        pose.pose.position.y = target.y;
        pose.pose.position.z = target.z;
        local_pos_pub_->publish(pose);

        // Check if the drone has reached the desired position
        if (reached_waypoint(target)) {
            RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu: [%f, %f, %f]", current_waypoint_, target.x, target.y, target.z);
            execute_task(target.task);
            
            current_waypoint_++;
        }

        // Continue to the next waypoint
        navigate_to_waypoint();
    }

    bool reached_waypoint(const Waypoint& waypoint) {
        
        // Implement logic to check if the current drone position is within 'precision' of the waypoint
        // Example: compare the current position with the waypoint coordinates
        // For now, this is just a placeholder:
        return true;  // Replace this with actual distance check
    }

    void execute_task(const std::string& task) {
        if (task == "takeoff") {
            // Call MAVROS takeoff service
            RCLCPP_INFO(this->get_logger(), "Executing task: Takeoff");
        } else if (task == "land") {
            // Call MAVROS land service
            RCLCPP_INFO(this->get_logger(), "Executing task: Land");
        } else if (task == "yaw180") {
            // Implement yaw command
            RCLCPP_INFO(this->get_logger(), "Executing task: Yaw 180");
        } else if (task == "yaw90") {
            // Implement yaw command
            RCLCPP_INFO(this->get_logger(), "Executing task: Yaw 90");
        } else if (task == "landtakeoff") {
            // Implement landing and takeoff
            RCLCPP_INFO(this->get_logger(), "Executing task: Land and Takeoff");
        }
    }
    void print_trajectory()
    {
        std::cout << "Trajectory:" << std::endl;
        for (const auto &waypoint : trajectory_)
        {
            std::cout << "X: " << waypoint.x << ", Y: " << waypoint.y << ", Z: " << waypoint.z << std::endl;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FloodFillDroneControl>());
    rclcpp::shutdown();
    return 0;
}
