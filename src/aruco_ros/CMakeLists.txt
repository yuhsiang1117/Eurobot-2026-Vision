cmake_minimum_required(VERSION 3.8)
project(aruco_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(aruco_detector_node src/aruco_detector_node.cpp)
ament_target_dependencies(aruco_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  geometry_msgs
)

add_executable(robot_detector_node src/robot_detector_node.cpp)
ament_target_dependencies(robot_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  geometry_msgs
)

target_link_libraries(
  aruco_detector_node
  ${OpenCV_LIBRARIES}
)

target_link_libraries(
  robot_detector_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  aruco_detector_node
  robot_detector_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
