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
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlebot3_msgs REQUIRED)

add_executable(patrol_nav_node src/patrol.cpp)
ament_target_dependencies(patrol_nav_node
  rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_geometry_msgs std_msgs turtlebot3_msgs
)
target_link_libraries(patrol_nav_node yaml-cpp)
install(TARGETS patrol_nav_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

add_executable(location_node src/location.cpp)
ament_target_dependencies(location_node
  rclcpp geometry_msgs
)
install(TARGETS location_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(fire src/fire.cpp)
target_include_directories(fire PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(fire ${OpenCV_LIBS})
ament_target_dependencies(fire rclcpp sensor_msgs cv_bridge)
install(TARGETS fire
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(person src/person.cpp)
target_include_directories(person PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(person ${OpenCV_LIBS})
ament_target_dependencies(person rclcpp sensor_msgs cv_bridge)
install(TARGETS person
  DESTINATION lib/${PROJECT_NAME}
)

# detection (std_msgs, geometry_msgs 추가)
add_executable(detection src/detection.cpp)
target_include_directories(detection PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(detection ${OpenCV_LIBS})
ament_target_dependencies(detection
  rclcpp sensor_msgs cv_bridge std_msgs geometry_msgs
)
install(TARGETS detection
  DESTINATION lib/${PROJECT_NAME}
)



add_executable(register_face src/register_face.cpp)
target_include_directories(register_face PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(register_face ${OpenCV_LIBS})
ament_target_dependencies(register_face
  rclcpp sensor_msgs cv_bridge
)
install(TARGETS register_face
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(test_patrol_nav src/test_patrol.cpp)
ament_target_dependencies(test_patrol_nav
  rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_geometry_msgs std_msgs turtlebot3_msgs
)
target_link_libraries(test_patrol_nav yaml-cpp)
install(TARGETS test_patrol_nav
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(test_detection src/test_detection.cpp)
target_include_directories(test_detection PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(test_detection ${OpenCV_LIBS})
ament_target_dependencies(test_detection
  rclcpp sensor_msgs cv_bridge std_msgs geometry_msgs
)

install(TARGETS test_detection
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(test_detection2 src/test_detection2.cpp)
target_include_directories(test_detection2 PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(test_detection2 ${OpenCV_LIBS})
ament_target_dependencies(test_detection2
  rclcpp sensor_msgs cv_bridge std_msgs geometry_msgs
)
install(TARGETS test_detection2
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()