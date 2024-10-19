#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "load_pcd_node.hpp"
class DroneNavigator : public rclcpp::Node
{
public:
    DroneNavigator();

    void calculatePath(const std::vector<Waypoint> &waypoints);

private:
    void checkCollision(const Waypoint &wp);
    void sendGoal(const geometry_msgs::msg::PoseStamped &goal_pose);

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
};
