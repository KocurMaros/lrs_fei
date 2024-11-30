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
    void loadMapPCD(const std::string &pcd_file, int slices);
    std::vector<std::string> generateMapFilenames();
    std::vector<Waypoint> loadWaypoints(const std::string &filename);
    nav_msgs::msg::OccupancyGrid getOccupancyGrid() { return map; }
    void fromPCD(double altitude);
    void saveSliceWithMarker(const nav_msgs::msg::OccupancyGrid &map, const std::string &filename, double marker_x, double marker_y);
private:
    int slices_;
    double max_height_;
    nav_msgs::msg::OccupancyGrid map;
    std::vector<nav_msgs::msg::OccupancyGrid> maps;
    void loadPGM(const std::string &pgm_file, nav_msgs::msg::OccupancyGrid &map);
    void loadPCDToMultiLayerGrid(const std::string &pcd_file, 
                                           std::vector<nav_msgs::msg::OccupancyGrid> &maps,
                                           int num_slices);
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    std::vector<Waypoint> waypoints_;
};

#endif // PGM_MAP_LOADER_HPP
