// pgm_map_loader.hpp

#ifndef PGM_MAP_LOADER_HPP
#define PGM_MAP_LOADER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <string>
#include <vector>
struct Waypoint {
    double x, y, z;
    std::string precision;
    std::string task;
};
class PGMMapLoader : public rclcpp::Node
{
public:
    PGMMapLoader();
    void loadMap(const std::string &pgm_file);
    std::vector<std::string> generateMapFilenames();
    std::vector<Waypoint> loadWaypoints(const std::string &filename);

    nav_msgs::msg::OccupancyGrid getOccupancyGrid() { return map; }
private:
    nav_msgs::msg::OccupancyGrid map;
    void loadPGM(const std::string &pgm_file, nav_msgs::msg::OccupancyGrid &map);
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    std::vector<Waypoint> waypoints_;
};

#endif // PGM_MAP_LOADER_HPP
