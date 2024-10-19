#include "load_pcd_node.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <sstream>

LoadPCDNode::LoadPCDNode() : Node("load_pcd_node")
{
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_topic", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&LoadPCDNode::publishMap, this));
}

void LoadPCDNode::publishMap()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("src/LRS-FEI/maps/FEI_LRS_PCD/map.pcd", cloud) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Couldn't read the map file");
        return;
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = "map";
    publisher_->publish(cloud_msg);
}

std::vector<Waypoint> LoadPCDNode::loadWaypoints(const std::string &filename)
{
    std::vector<Waypoint> waypoints;
    std::ifstream file(filename);
    if (!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Could not open waypoint file: %s", filename.c_str());
        return waypoints;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        Waypoint wp;

        // Read the line and split by comma
        std::string token;
        std::getline(ss, token, ',');
        wp.x = std::stof(token); // Convert to float

        std::getline(ss, token, ',');
        wp.y = std::stof(token); // Convert to float

        std::getline(ss, token, ',');
        wp.z = std::stof(token); // Convert to float

        std::getline(ss, wp.precision, ','); // Read precision directly
        std::getline(ss, wp.task); // Read task directly

        // Validate parsing
        if (ss.fail()) {
            RCLCPP_WARN(this->get_logger(), "Could not parse line: %s", line.c_str());
        } else {
            waypoints.push_back(wp);
        }
    }

    waypoints_ = waypoints; // Store waypoints as a member variable
    publishTrajectory();
    return waypoints;
}



void LoadPCDNode::publishTrajectory()
{
    std::cout << "Publishing trajectory" << std::endl;
    if (waypoints_.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No waypoints loaded to publish.");
        return;
    }
    visualization_msgs::msg::Marker line_strip;
    line_strip.header.frame_id = "map";
    line_strip.header.stamp = this->now();
    line_strip.ns = "trajectory";
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 0;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.scale.x = 0.1; // Line width

    // Set the color (red)
    line_strip.color.r = 1.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 0.0;
    line_strip.color.a = 1.0;

    // Add the waypoints to the line strip
    for (const auto& wp : waypoints_)
    {
        geometry_msgs::msg::Point p;
        p.x = wp.x;
        p.y = wp.y;
        p.z = wp.z;
        line_strip.points.push_back(p);
    }

    marker_pub_->publish(line_strip);
}