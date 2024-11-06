#include "pgm_map_loader.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <iomanip>

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
    int n = 3; // Number of additional occupied squares around each occupied cell
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
