#ifndef DRONE_NAVIGATION_HPP
#define DRONE_NAVIGATION_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>

// Declare the function so it can be used in other files
nav_msgs::msg::Path generatePath(const nav_msgs::msg::OccupancyGrid& map, 
                                 const geometry_msgs::msg::Pose& start, 
                                 const geometry_msgs::msg::Pose& goal);

#endif // DRONE_NAVIGATION_HPP
