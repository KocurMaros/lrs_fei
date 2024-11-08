#ifndef LOAD_PCD_NODE_HPP
#define LOAD_PCD_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <string>

struct Waypoint {
    double x, y, z;
    std::string precision;
    std::string task;
};
class LoadPCDNode : public rclcpp::Node
{
public:
    LoadPCDNode();
    std::vector<Waypoint> loadWaypoints(const std::string &filename);
    void visualizeTrajectory(const std::vector<Waypoint>& waypoints);
    void publishTrajectory();
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_vis_;
    std::vector<Waypoint> waypoints_;
    void publishMap();
};

#endif // LOAD_PCD_NODE_HPP