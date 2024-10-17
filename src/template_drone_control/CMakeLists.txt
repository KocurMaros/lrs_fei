cmake_minimum_required(VERSION 3.5)
project(template_drone_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # For PointCloud2 messages
find_package(pcl_conversions REQUIRED)  # For PCL conversions
find_package(PCL REQUIRED)  # For PCD loading

# Declare the first C++ executable (drone control)
add_executable(template_drone_control_node src/template_drone_control_node.cpp)
ament_target_dependencies(template_drone_control_node rclcpp geometry_msgs mavros_msgs)

# Declare the second C++ executable (PCD loader)
add_executable(pcd_loader_node src/pcd_loader.cpp)
ament_target_dependencies(pcd_loader_node rclcpp sensor_msgs pcl_conversions)

add_executable(flood_fill_node src/flood_fill_node.cpp)
ament_target_dependencies(flood_fill_node rclcpp mavros_msgs geometry_msgs sensor_msgs pcl_conversions)

# Link PCL libraries
target_link_libraries(flood_fill_node ${PCL_LIBRARIES})

# Install the executables
install(TARGETS
  template_drone_control_node
  pcd_loader_node
  flood_fill_node
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package
ament_package()
