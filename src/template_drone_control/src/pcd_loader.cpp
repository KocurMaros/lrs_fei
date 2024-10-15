#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class MapPublisher : public rclcpp::Node
{
public:
    MapPublisher() : Node("map_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&MapPublisher::publishMap, this));
    }

private:
    void publishMap()
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

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}