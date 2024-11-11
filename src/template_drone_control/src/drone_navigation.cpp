#include <vector>
#include <queue>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "rclcpp/time.hpp"
#include <iostream>
#include <memory>
#include <functional>


struct Node
{
    int x, y;
    double cost;
    std::shared_ptr<Node> parent;

    Node(int x_, int y_, double cost_ = 0.0, std::shared_ptr<Node> parent_ = nullptr)
        : x(x_), y(y_), cost(cost_), parent(parent_) {}
};


// Manhattan distance heuristic
double heuristic(int x1, int y1, int x2, int y2)
{
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

bool isValid(int x, int y, const nav_msgs::msg::OccupancyGrid& map)
{
    int width = map.info.width;
    int height = map.info.height;
    if (x < 0 || x >= width || y < 0 || y >= height)
        return false;

    int index = y * width + x;
    return map.data[index] == 0; // Check if free space
}

std::vector<geometry_msgs::msg::PoseStamped> aStarPathfinding(
    const nav_msgs::msg::OccupancyGrid& map,
    int start_x, int start_y,
    int goal_x, int goal_y)
{
    std::cout << "Start: " << start_x << ", " << start_y << std::endl;
    std::cout << "Goal: " << goal_x << ", " << goal_y << std::endl;

    struct CompareNode
    {
        bool operator()(const std::pair<double, std::shared_ptr<Node>>& a, const std::pair<double, std::shared_ptr<Node>>& b) const
        {
            return a.first > b.first; // Min-heap
        }
    };

    std::priority_queue<
        std::pair<double, std::shared_ptr<Node>>,
        std::vector<std::pair<double, std::shared_ptr<Node>>>,
        CompareNode
    > open_list;

    int width = map.info.width;
    int height = map.info.height;

    std::vector<std::vector<bool>> visited(width, std::vector<bool>(height, false));

    auto start_node = std::make_shared<Node>(start_x, start_y);
    open_list.push({0.0, start_node});

    std::shared_ptr<Node> goal_node = nullptr;

    while (!open_list.empty())
    {
        auto current = open_list.top().second;
        open_list.pop();

        if (visited[current->x][current->y])
            continue;
        visited[current->x][current->y] = true;

        // Check if we reached the goal
        if (current->x == goal_x && current->y == goal_y)
        {
            goal_node = current;
            break;
        }

        // Explore neighbors (4-connected grid)
        std::vector<std::pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
        for (auto& dir : directions)
        {
            int new_x = current->x + dir.first;
            int new_y = current->y + dir.second;

            if (isValid(new_x, new_y, map) && !visited[new_x][new_y])
            {
                double new_cost = current->cost + 1.0;
                auto neighbor = std::make_shared<Node>(new_x, new_y, new_cost, current);
                double priority = new_cost + heuristic(new_x, new_y, goal_x, goal_y);
                open_list.push({priority, neighbor});
            }
        }
    }

    std::vector<geometry_msgs::msg::PoseStamped> path;
    if (goal_node != nullptr)
    {
        // Backtrack to create the path
        for (auto n = goal_node; n != nullptr; n = n->parent)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = n->x * map.info.resolution + map.info.origin.position.x;
            pose.pose.position.y = n->y * map.info.resolution + map.info.origin.position.y;
            
            path.push_back(pose);
        }
        std::reverse(path.begin(), path.end());
    }
    else
    {
        std::cout << "No path found." << std::endl;
    }

    return path;
}
    
nav_msgs::msg::Path generatePath(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Pose& drone_position,
    const geometry_msgs::msg::Pose& goal_position)
{
    int start_x = static_cast<int>((drone_position.position.x - map.info.origin.position.x) / map.info.resolution);
    int start_y = static_cast<int>((drone_position.position.y - map.info.origin.position.y) / map.info.resolution);

    int goal_x = static_cast<int>((goal_position.position.x - map.info.origin.position.x) / map.info.resolution);
    int goal_y = static_cast<int>((goal_position.position.y - map.info.origin.position.y) / map.info.resolution);

    std::vector<geometry_msgs::msg::PoseStamped> path_points = aStarPathfinding(map, start_x, start_y, goal_x, goal_y);

    std::vector<geometry_msgs::msg::PoseStamped> optimized_path;
    if (!path_points.empty())
    {
        optimized_path.push_back(path_points.front()); // Start point
        const double tolerance = 0.3; // Adjust this threshold based on acceptable offset tolerance

        for (size_t i = 1; i < path_points.size() - 1; ++i)
        {
            auto& prev = path_points[i - 1].pose.position;
            auto& curr = path_points[i].pose.position;
            auto& next = path_points[i + 1].pose.position;

            double dx1 = curr.x - prev.x;
            double dy1 = curr.y - prev.y;
            double dx2 = next.x - curr.x;
            double dy2 = next.y - curr.y;

            
            // Check if the movement is approximately straight or diagonal
            bool is_approx_straight = !(dx1 * dy2 != dy1 * dx2);

            bool is_approx_diagonal = (std::abs(dx1) > tolerance && std::abs(dy1) > tolerance) &&
                                      (std::abs(dx1 - dx2) <= tolerance && std::abs(dy1 - dy2) <= tolerance);

            // Keep points only if there's a change in movement type
            if ((!is_approx_diagonal && !is_approx_straight) || 
                (is_approx_straight && (dx1 * dx2 < 0 || dy1 * dy2 < 0))) // Change in direction
            {
                optimized_path.push_back(path_points[i]);
            }
        }

        optimized_path.push_back(path_points.back()); // Goal point
    }
    
    path_points = optimized_path;
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = rclcpp::Time();
    path.poses = path_points;

    return path;
}
