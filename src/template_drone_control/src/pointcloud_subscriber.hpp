
#ifndef POINTCLOUD_SUBSCRIBER_HPP
#define POINTCLOUD_SUBSCRIBER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class PointCloudSubscriber : public rclcpp::Node
{
public:
    PointCloudSubscriber();

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

#endif // POINTCLOUD_SUBSCRIBER_HPP
