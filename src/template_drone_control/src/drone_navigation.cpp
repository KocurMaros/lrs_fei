#include "drone_navigation.hpp"

DroneNavigator::DroneNavigator() : Node("drone_navigator")
{
    waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("waypoint_topic", 10);
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "navigate_to_pose");
}

void DroneNavigator::checkCollision(const Waypoint &wp)
{
    // Implement collision checking logic here
}

void DroneNavigator::sendGoal(const geometry_msgs::msg::PoseStamped &goal_pose)
{
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available!");
        return;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = goal_pose;

    RCLCPP_INFO(this->get_logger(), "Sending goal...");
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
        [](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(rclcpp::get_logger("DroneNavigator"), "Goal reached!");
            } else {
                RCLCPP_WARN(rclcpp::get_logger("DroneNavigator"), "Goal failed with code: %d", result.code);
            }
        };

    action_client_->async_send_goal(goal_msg, send_goal_options);
}

void DroneNavigator::calculatePath(const std::vector<Waypoint> &waypoints)
{
    for (const auto &wp : waypoints) {
        // Create a PoseStamped message for the waypoint
        geometry_msgs::msg::PoseStamped waypoint_msg;
        waypoint_msg.header.frame_id = "map"; // Adjust frame if necessary
        waypoint_msg.pose.position.x = wp.x;
        waypoint_msg.pose.position.y = wp.y;
        waypoint_msg.pose.position.z = wp.z;

        // Check for collision
        checkCollision(wp);

        // Publish the waypoint (optional)
        waypoint_pub_->publish(waypoint_msg);

        // Send the goal to the Navigation2 action server
        sendGoal(waypoint_msg);

        // Optional: Wait for a brief period to ensure the drone has time to navigate to the waypoint
        std::this_thread::sleep_for(std::chrono::seconds(5)); // Adjust as needed
    }
}
