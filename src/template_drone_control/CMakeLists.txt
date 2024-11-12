cmake_minimum_required(VERSION 3.5)
project(template_drone_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(OpenCV REQUIRED)
find_package(YAML-CPP REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(dronestappen_control_node
  src/template_drone_control_node.cpp
  src/drone_navigation.cpp
  src/load_pcd_node.cpp
  src/pgm_map_loader.cpp)

ament_target_dependencies(dronestappen_control_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  mavros_msgs
  nav2_msgs
  rclcpp_action
  pcl_conversions
  OpenCV
  YAML-CPP
  tf2
  tf2_geometry_msgs
  PCL)

target_link_libraries(dronestappen_control_node
  ${YAML_CPP_LIBRARIES})

install(TARGETS
  dronestappen_control_node
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()