cmake_minimum_required(VERSION 3.8)
project(pt_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(patrol_nav_node src/patrol.cpp)

ament_target_dependencies(patrol_nav_node
  rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_geometry_msgs
)

target_link_libraries(patrol_nav_node yaml-cpp)

install(TARGETS patrol_nav_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()