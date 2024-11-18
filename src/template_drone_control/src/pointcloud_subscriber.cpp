#include "pointcloud_subscriber.hpp"

// Implementation of PointCloudSubscriber methods (no main function here)
PointCloudSubscriber::PointCloudSubscriber() : Node("pointcloud_subscriber")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/stereo_camera/points", 10,
        std::bind(&PointCloudSubscriber::pointCloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "PointCloud Subscriber Initialized.");
}

void PointCloudSubscriber::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received PointCloud with width: %u, height: %u",
    //             msg->width, msg->height);
}
