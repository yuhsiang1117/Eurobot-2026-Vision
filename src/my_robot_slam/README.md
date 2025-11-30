# my_robot_slam

This package provides the necessary launch files and configurations for Simultaneous Localization and Mapping (SLAM) using RTAB-MAP on ROS 2 Humble with an Intel RealSense D435 camera and a 2D Lidar.

## 1. System Dependencies

The following packages must be installed on your host system:

```bash
sudo apt update
sudo apt install -y ros-humble-realsense2-camera \
                    ros-humble-rtabmap-ros \
                    ros-humble-rplidar-ros \
                    ros-humble-tf2-tools \
                    ros-humble-rmw-cyclonedds-cpp # Recommended for high bandwidth
```

## 2. Build Instructions

Navigate to your workspace root and build the `my_robot_slam` package:

```bash
cd ~/vision_ws
colcon build --symlink-install --packages-select my_robot_slam
source install/setup.bash
```

## 3. Run Instructions

After building and sourcing your workspace, launch the mapping pipeline:

```bash
ros2 launch my_robot_slam mapping_launch.py
```

## 4. Troubleshooting Checklist

*   **Check TF Tree:** Run `ros2 run tf2_tools view_frames`. Requirement: Must see `map -> odom -> base_link -> camera_link`/`lidar_link`. Note: If `odom -> base_link` is missing, ensure the robot chassis driver is running.
*   **Check Topics:** Run `ros2 topic list`. Requirement: Ensure `/camera/camera/aligned_depth_to_color/image_raw` and `/scan` exist and are publishing data.

```