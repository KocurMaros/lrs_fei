#include "pgm_map_loader.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <fstream>
#include <iostream>
#include <sstream>

PGMMapLoader::PGMMapLoader() : Node("pgm_map_loader")
{
    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
}

void PGMMapLoader::loadMap(const std::string &pgm_file)
{
    nav_msgs::msg::OccupancyGrid map;
    
    // Load and publish the map
    loadPGM(pgm_file, map);
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
    double resolution = 13.0 / 366;  // Known size: 13 meters wide, 366 pixels
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
    map.data.resize(width * height);
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
                map.data[index] = 100;  // Occupied
            }
            else if (value == 255)
            {
                map.data[index] = 0;    // Free
            }
            else
            {
                map.data[index] = -1;   // Unknown
            }
        }
    }
    std::cout << "Map loaded and published" << std::endl;
    std::cout << "Map resolution: " << resolution << std::endl;
    std::cout << "Map origin: " << origin[0] << ", " << origin[1] << ", " << origin[2] << std::endl;
    std::cout << "Map width: " << map.info.width << std::endl;
    std::cout << "Map height: " << map.info.height << std::endl;
    
}

std::vector<std::string> PGMMapLoader::generateMapFilenames()
{
    std::vector<std::string> map_filenames;
    std::string base_path = "src/LRS-FEI/maps/FEI_LRS_2D/map_";

    // Generate file names from 025 to 225 with a step of 25
    for (int i = 25; i <= 225; i += 25)
    {
        // Create a string stream to format the number as 3 digits with leading zeros
        std::stringstream ss;
        ss << std::setw(3) << std::setfill('0') << i;

        // Append the formatted number to the base path
        std::string map_filename = base_path + ss.str() + ".pgm";
        map_filenames.push_back(map_filename);
    }

    return map_filenames;
}
