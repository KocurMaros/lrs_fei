#include "pgm_map_loader.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <iomanip>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <limits>
#include <algorithm>

PGMMapLoader::PGMMapLoader() : Node("pgm_map_loader")
{
    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
}

void PGMMapLoader::loadMap(const std::string &pgm_file)
{
    // Load and publish the map
    loadPGM(pgm_file, map);
    map_publisher_->publish(map);
}
void PGMMapLoader::loadMapPCD(const std::string &pcd_file, int slices)
{
    slices_ = slices;
    // Load and publish the map
    loadPCDToMultiLayerGrid(pcd_file, maps, slices_);
    map_publisher_->publish(maps[0]);
}
void PGMMapLoader::fromPCD(double altitude){
    double slice_thickness = max_height_ / slices_;
    int slice_index = static_cast<int>(std::floor(altitude / slice_thickness));
    // Clamp slice index to valid range [0, num_slices - 1]
    slice_index = std::max(0, std::min(slice_index, slices_ - 1));
    std::cout << "Slice index: " << slice_index << "\n";
    map = maps[slice_index];
    map_publisher_->publish(map);
}
void PGMMapLoader::loadPGM(const std::string &pgm_file, nav_msgs::msg::OccupancyGrid &map)
{
    std::ifstream infile(pgm_file);
    if (!infile)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open PGM file: %s", pgm_file.c_str());
        return;
    }

    std::string line;
    int width, height;
    double resolution = 0.05;  // Known size: 13 meters wide, 366 pixels
    std::vector<double> origin = {0.0, 0.0, 0.0};

    // Ignore the first line (P2)
    std::getline(infile, line);

    // Get width and height from second line
    infile >> width >> height;

    // Ignore the third line (max pixel value)
    std::getline(infile, line);
    std::getline(infile, line);

    // Initialize the occupancy grid map
    map.info.resolution = resolution;
    map.info.width = width;
    map.info.height = height;
    map.info.origin.position.x = origin[0];
    map.info.origin.position.y = origin[1];
    map.info.origin.position.z = origin[2];
    map.info.origin.orientation.w = 1.0;

    // Read the data and convert to occupancy grid
    std::vector<int8_t> original_map_data(width * height);

    // Read the PGM file into original_map_data
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int value;
            infile >> value;

            int index = y * width + x;

            // Map PGM values to occupancy (-1 for unknown, 0 for free, 100 for occupied)
            if (value == 0)
            {
                original_map_data[index] = 100;  // Occupied
            }
            else if (value == 255)
            {
                original_map_data[index] = 0;    // Free
            }
            else
            {
                original_map_data[index] = -1;   // Unknown
            }
        }
    }

    // Write the original map data to a text file before inflation
    std::ofstream original_outfile("original_map.txt");
    if (original_outfile.is_open())
    {
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                int index = y * width + x;
                int value = original_map_data[index];

                // Write the value to the file with proper formatting
                if (value == 100)
                    original_outfile << "1 ";  // Represent occupied cells with '1'
                else if (value == 0)
                    original_outfile << "0 ";  // Represent free cells with '0'
                else
                    original_outfile << "-1 "; // Represent unknown cells with '-1'
            }
            original_outfile << "\n"; // New line at the end of each row
        }
        original_outfile.close();
        std::cout << "Original map data written to original_map.txt" << std::endl;
    }
    else
    {
        std::cerr << "Unable to open file for writing original map data." << std::endl;
    }

    // Inflate obstacles
    int n = 4; // Number of additional occupied squares around each occupied cell
    std::vector<int8_t> inflated_map_data = original_map_data; // Start with the original map data

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int index = y * width + x;
            if (original_map_data[index] == 100) // Occupied cell
            {
                // Mark neighboring cells within n cells
                for (int dy = -n; dy <= n; ++dy)
                {
                    for (int dx = -n; dx <= n; ++dx)
                    {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height)
                        {
                            int nindex = ny * width + nx;
                            inflated_map_data[nindex] = 100; // Mark as occupied
                        }
                    }
                }
            }
        }
    }

    // Assign the inflated map data to the occupancy grid
    map.data = inflated_map_data;

    // Write the inflated map data to a text file for verification
    std::ofstream inflated_outfile("inflated_map.txt");
    if (inflated_outfile.is_open())
    {
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                int index = y * width + x;
                int value = map.data[index];

                // Write the value to the file with proper formatting
                if (value == 100)
                    inflated_outfile << "1 ";  // Represent occupied cells with '1'
                else if (value == 0)
                    inflated_outfile << "0 ";  // Represent free cells with '0'
                else
                    inflated_outfile << "-1 "; // Represent unknown cells with '-1'
            }
            inflated_outfile << "\n"; // New line at the end of each row
        }
        inflated_outfile.close();
        std::cout << "Inflated map data written to inflated_map.txt" << std::endl;
    }
    else
    {
        std::cerr << "Unable to open file for writing inflated map data." << std::endl;
    }

    std::cout << "Map loaded and published" << std::endl;
    std::cout << "Map resolution: " << resolution << std::endl;
    std::cout << "Map origin: " << origin[0] << ", " << origin[1] << ", " << origin[2] << std::endl;
    std::cout << "Map width: " << map.info.width << std::endl;
    std::cout << "Map height: " << map.info.height << std::endl;
}


void PGMMapLoader::loadPCDToMultiLayerGrid(const std::string &pcd_file, 
                                           std::vector<nav_msgs::msg::OccupancyGrid> &maps,
                                           int num_slices)
{
    // Load the PCD file using PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", pcd_file.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "PCD file loaded. Points: %zu", cloud->size());

    // Find the bounding box of the point cloud
    double x_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::lowest();
    double y_min = std::numeric_limits<double>::max();
    double y_max = std::numeric_limits<double>::lowest();
    double z_min = std::numeric_limits<double>::max();
    double z_max = std::numeric_limits<double>::lowest();

    for (const auto &point : cloud->points)
    {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
        {
            x_min = std::min(x_min, static_cast<double>(point.x));
            x_max = std::max(x_max, static_cast<double>(point.x));
            y_min = std::min(y_min, static_cast<double>(point.y));
            y_max = std::max(y_max, static_cast<double>(point.y));
            z_min = std::min(z_min, static_cast<double>(point.z));
            z_max = std::max(z_max, static_cast<double>(point.z));
        }
    }

    RCLCPP_INFO(this->get_logger(), "Point cloud bounding box:");
    RCLCPP_INFO(this->get_logger(), "  X: [%f, %f]", x_min, x_max);
    RCLCPP_INFO(this->get_logger(), "  Y: [%f, %f]", y_min, y_max);
    RCLCPP_INFO(this->get_logger(), "  Z: [%f, %f]", z_min, z_max);
    max_height_ = z_max;
    // Derived parameters
    double resolution = 0.05;  // Fixed resolution: 5 cm per grid cell
    double map_width = x_max - x_min;
    double map_height = y_max - y_min;
    double slice_thickness = (z_max - z_min) / num_slices;

    // Calculate the number of cells in the grid
    int grid_width = static_cast<int>(std::ceil(map_width / resolution));
    int grid_height = static_cast<int>(std::ceil(map_height / resolution));

    // Initialize the occupancy grids for each slice
    maps.resize(num_slices);

    for (int slice = 0; slice < num_slices; ++slice)
    {
        // Initialize the occupancy grid for this slice
        nav_msgs::msg::OccupancyGrid &map_ = maps[slice];
        map_.info.resolution = resolution;
        map_.info.width = grid_width;
        map_.info.height = grid_height;
        map_.info.origin.position.x = 0;
        map_.info.origin.position.y = 0;
        map_.info.origin.position.z = z_min + slice * slice_thickness; // Set origin height for this slice
        map_.info.origin.orientation.w = 1.0;

        // Initialize the map_ data
        map_.data.resize(grid_width * grid_height, -1); // -1 for unknown cells
    }

    // Process the point cloud and project onto 2D slices
    for (const auto &point : cloud->points)
    {
        // Skip invalid points
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            continue;

        // Determine the slice for this point based on its z-value
        int slice_index = static_cast<int>(std::floor((point.z - z_min) / slice_thickness));
        if (slice_index < 0 || slice_index >= num_slices)
            continue;

        nav_msgs::msg::OccupancyGrid &map_ = maps[slice_index];

        // Convert the 3D point to 2D grid coordinates
        int grid_x = static_cast<int>(std::floor((point.x - x_min) / resolution));
        int grid_y = static_cast<int>(std::floor((point.y - y_min) / resolution));

        // Ensure the point is within the bounds of the grid
        if (grid_x >= 0 && grid_x < grid_width && grid_y >= 0 && grid_y < grid_height)
        {
            int index = grid_y * grid_width + grid_x;
            map_.data[index] = 100; // Mark cell as occupied
        }
    }

    // Optionally, write each slice's data to a text file for verification
    for (int slice = 0; slice < num_slices; ++slice)
    {
        const nav_msgs::msg::OccupancyGrid &map_ = maps[slice];
        std::ofstream slice_outfile("slice_map_" + std::to_string(slice) + ".txt");
        if (slice_outfile.is_open())
        {
            for (int y = 0; y < grid_height; ++y)
            {
                for (int x = 0; x < grid_width; ++x)
                {
                    int index = y * grid_width + x;
                    slice_outfile << (map_.data[index] == 100 ? "1 " : "0 ");
                }
                slice_outfile << "\n";
            }
            slice_outfile.close();
            RCLCPP_INFO(this->get_logger(), "Slice %d map_ data written to slice_map_%d.txt", slice, slice);
        }
    }

    RCLCPP_INFO(this->get_logger(), "3D point cloud projected into %d 2D occupancy grids.", num_slices);
}

std::vector<std::string> PGMMapLoader::generateMapFilenames()
{
    std::vector<std::string> map_heights;
    map_heights.push_back("025");
    map_heights.push_back("075");
    map_heights.push_back("080");
    map_heights.push_back("100");
    map_heights.push_back("125");
    map_heights.push_back("150");
    map_heights.push_back("175");
    map_heights.push_back("180");
    map_heights.push_back("200");
    map_heights.push_back("225");
    
    std::vector<std::string> map_filenames;
    std::string base_path = "src/LRS-FEI/maps/FEI_LRS_2D/map_";


    for(auto height : map_heights){
        std::string map_filename = base_path + height + ".pgm";
        map_filenames.push_back(map_filename);
    }
    // Generate file names from 025 to 225 with a step of 25
    return map_filenames;
}
std::vector<Waypoint> PGMMapLoader::loadWaypoints(const std::string &filename)
{
    std::vector<Waypoint> waypoints;
    std::ifstream file(filename);
    if (!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Could not open waypoint file: %s", filename.c_str());
        return waypoints;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        Waypoint wp;

        // Read the line and split by comma
        std::string token;
        std::getline(ss, token, ',');
        wp.x = std::stof(token); // Convert to float

        std::getline(ss, token, ',');
        wp.y = std::stof(token); // Convert to float

        std::getline(ss, token, ',');
        wp.z = std::stof(token); // Convert to float

        std::getline(ss, wp.precision, ','); // Read precision directly
        std::getline(ss, wp.task); // Read task directly

        // Validate parsing
        if (ss.fail()) {
            RCLCPP_WARN(this->get_logger(), "Could not parse line: %s", line.c_str());
        } else {
            waypoints.push_back(wp);
        }
    }

    waypoints_ = waypoints; // Store waypoints as a member variable
    return waypoints;
}

void PGMMapLoader::saveSliceWithMarker(const nav_msgs::msg::OccupancyGrid &map, const std::string &filename, double marker_x, double marker_y)
{
    int grid_width = map.info.width;
    int grid_height = map.info.height;
    double resolution = map.info.resolution;
    double origin_x = map.info.origin.position.x;
    double origin_y = map.info.origin.position.y;

    // Convert world coordinates (marker_x, marker_y) to grid indices
    int grid_x = static_cast<int>((marker_x - origin_x) / resolution);
    int grid_y = static_cast<int>((marker_y - origin_y) / resolution);

    // Write the updated grid data to a text file
    std::ofstream slice_outfile(filename);
    if (slice_outfile.is_open())
    {
        for (int y = 0; y < grid_height; ++y)
        {
            for (int x = 0; x < grid_width; ++x)
            {
                int index = y * grid_width + x;
                if(y == grid_y && x == grid_x)
                    slice_outfile << static_cast<int>(2) << " ";
                else    
                    slice_outfile << static_cast<int>(map.data[index]) << " ";
            }
            slice_outfile << "\n";
        }
        slice_outfile.close();
        std::cout << "Map slice with marker saved to " << filename << std::endl;
    }
    else
    {
        std::cerr << "Unable to open file for writing: " << filename << std::endl;
    }
}
