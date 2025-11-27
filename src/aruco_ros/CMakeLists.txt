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
add_executable(camera_calibrator_node src/calibrate_camera.cpp)

ament_target_dependencies(aruco_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  geometry_msgs
)

ament_target_dependencies(camera_calibrator_node
  rclcpp
  sensor_msgs
  cv_bridge
  ament_index_cpp
)

target_link_libraries(aruco_detector_node
  ${OpenCV_LIBRARIES}
)

target_link_libraries(camera_calibrator_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  aruco_detector_node
  camera_calibrator_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
