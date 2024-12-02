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
#include <fstream>
#include <string>
#include <geometry_msgs/msg/pose.hpp>
int iteration = 0;
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
    goal_y += 1;
    double posX, posY;
    std::cout << "Start: " << start_x << ", " << start_y << std::endl;
    posX = start_x*0.05;
    posY = start_y*0.05;
    std::cout << "Start: " << posX << ", " << posY << std::endl;
    std::cout << "Goal: " << goal_x << ", " << goal_y << std::endl;
    posX = goal_x*0.05;
    posY = goal_y*0.05;
    std::cout << "Goal: " << posX << ", " << posY << std::endl;

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
#include <fstream>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

void save_path(const nav_msgs::msg::OccupancyGrid& map,
               const geometry_msgs::msg::Pose& drone_position,
               const geometry_msgs::msg::Pose& goal_position,
               std::vector<geometry_msgs::msg::PoseStamped> path_points) // Pass iteration as a reference to update externally
{
    // Generate filename with iteration
    std::string filename = "paths/path_map_" + std::to_string(iteration) + ".txt";
    iteration++;
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Unable to open file to save path data." << std::endl;
        return;
    }

    // Write header with goal and start positions
    file << "Start Position: x=" << drone_position.position.x
         << ", y=" << drone_position.position.y
         << ", z=" << drone_position.position.z << "\n";
    file << "Goal Position: x=" << goal_position.position.x
         << ", y=" << goal_position.position.y
         << ", z=" << goal_position.position.z << "\n\n";

    // Copy the map data
    std::vector<int8_t> grid_data = map.data;

    // Convert the drone position (start point) to grid indices
    int start_x = static_cast<int>((drone_position.position.x - map.info.origin.position.x) / map.info.resolution);
    int start_y = static_cast<int>((drone_position.position.y - map.info.origin.position.y) / map.info.resolution);
    if (start_x >= 0 && start_x < map.info.width && start_y >= 0 && start_y < map.info.height)
    {
        int start_index = start_y * map.info.width + start_x;
        grid_data[start_index] = 2; // Start point
    }

    // Convert the goal position (end point) to grid indices
    int goal_x = static_cast<int>((goal_position.position.x - map.info.origin.position.x) / map.info.resolution);
    int goal_y = static_cast<int>((goal_position.position.y - map.info.origin.position.y) / map.info.resolution);
    if (goal_x >= 0 && goal_x < map.info.width && goal_y >= 0 && goal_y < map.info.height)
    {
        int goal_index = goal_y * map.info.width + goal_x;
        grid_data[goal_index] = 0; // Goal point
    }

    // Mark the path points on the grid
    int point_in_path = 3;
    for (const auto& point : path_points)
    {
        int path_x = static_cast<int>((point.pose.position.x - map.info.origin.position.x) / map.info.resolution);
        int path_y = static_cast<int>((point.pose.position.y - map.info.origin.position.y) / map.info.resolution);

        if (path_x >= 0 && path_x < map.info.width && path_y >= 0 && path_y < map.info.height)
        {
            int path_index = path_y * map.info.width + path_x;
            grid_data[path_index] = point_in_path++; // Increment path point number
        }
    }

    // Write the grid data to the file
    for (int y = 0; y < map.info.height; ++y)
    {
        for (int x = 0; x < map.info.width; ++x)
        {
            int index = y * map.info.width + x;
            if (grid_data[index] == 0)
                file << static_cast<int>(-1) << " "; // Represent empty cells as -1
            else
                file << static_cast<int>(grid_data[index]) << " ";
        }
        file << "\n";
    }

    file.close();
    std::cout << "Path data saved to " << filename << std::endl;
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
    save_path(map, drone_position, goal_position, path_points);
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
