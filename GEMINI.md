# GEMINI.md - ROS 2 RTAB-MAP Implementation Guide

## Project Context
**Goal:** Implement SLAM (Simultaneous Localization and Mapping) using RTAB-MAP on ROS 2 Humble.
**Hardware:**
1.  **Robot Chassis:** Provides wheel odometry (Frame: `odom` -> `base_link`).
2.  **Camera:** Intel RealSense D435 (RGB-D).
3.  **Lidar:** 2D Laser Scanner (e.g., RPLidar).
**Software:** ROS 2 Humble (Ubuntu 22.04).

---

## 1. System Dependencies
The following packages must be installed on the host system via `apt`:

```bash
sudo apt update
sudo apt install -y ros-humble-realsense2-camera \
                    ros-humble-rtabmap-ros \
                    ros-humble-rplidar-ros \
                    ros-humble-tf2-tools \
                    ros-humble-rmw-cyclonedds-cpp # Recommended for high bandwidth

```
2. Workspace & Package Structure
We are creating a bringup package named my_robot_slam.

Target Directory Structure:

~/vision_ws/src/
└── my_robot_slam/
    ├── package.xml
    ├── setup.py
    └── launch/
        └── mapping_launch.py  <-- The Master Launch File


Action: Create Package
Bash

cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_slam
mkdir -p my_robot_slam/launch
3. Implementation Code
File: launch/mapping_launch.py
Description: This launch file orchestrates the Lidar, RealSense camera, Static Transforms, and RTAB-MAP node. It handles the critical synchronization between depth images and laser scans.

Python

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        
        # ----------------------------------------
        # 1. HARDWARE: RealSense D435
        # ----------------------------------------
        # We enable 'align_depth' to ensure the depth map perfectly overlaps the RGB image.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'
                ])
            ]),
            launch_arguments={
                'align_depth.enable': 'true',
                'pointcloud.enable': 'false', # RTAB-Map handles cloud generation
            }.items()
        ),

        # ----------------------------------------
        # 2. HARDWARE: Lidar (RPLidar Example)
        # ----------------------------------------
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            parameters=[{'frame_id': 'lidar_link'}]
        ),

        # ----------------------------------------
        # 3. STATIC TRANSFORMS (TF)
        # ----------------------------------------
        # Define where sensors are relative to the robot center (base_link).
        # Arguments: x y z yaw pitch roll parent child
        
        # Camera: 10cm forward
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.1', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
        ),
        # Lidar: 20cm up
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0.2', '0', '0', '0', 'base_link', 'lidar_link']
        ),

        # ----------------------------------------
        # 4. SLAM: RTAB-MAP
        # ----------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
                ])
            ]),
            launch_arguments={
                'frame_id': 'base_link',
                
                # Subscribe to both Depth Camera and Lidar
                'subscribe_depth': 'true',
                'subscribe_scan': 'true',
                
                # Topic Remappings (Standard RealSense Topics)
                'rgb_topic': '/camera/camera/color/image_raw',
                'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/camera/camera/color/camera_info',
                'scan_topic': '/scan',
                
                # Synchronization (CRITICAL for multi-sensor setups)
                'approx_sync': 'true',
                'queue_size': '20',
                
                # RTAB-Map Arguments
                # --delete_db_on_start: Resets the map every time (good for testing)
                'rtabmap_args': '--delete_db_on_start',
                
                # Visualization
                'rtabmap_viz': 'true',
            }.items()
        ),
    ])


4. Build & Run Instructions
Build:


cd ~/ros2_ws
colcon build --symlink-install --packages-select my_robot_slam
source install/setup.bash
Run:

Bash

ros2 launch my_robot_slam mapping_launch.py
5. Troubleshooting Checklist
Check TF Tree: Run: ros2 run tf2_tools view_frames Requirement: Must see map -> odom -> base_link -> camera_link/lidar_link. Note: If odom -> base_link is missing, ensure the robot chassis driver is running.

Check Topics: Run: ros2 topic list Requirement: Ensure /camera/camera/aligned_depth_to_color/image_raw and /scan exist and are publishing data.


### How to use this file
1.  **Save it** as `GEMINI.md` in your project root.
2.  **If using an AI Assistant (Cursor/VS Code Copilot):** Open the chat and reference this file (e.g., "@GEMINI.md generate the launch file").
3.  **If using manually:** Read Section 3 and copy the Python code into your `launch/mapping_launch.py` file.

**Next Step:** Once you have this file, would you like me to explain how to interpret the RTAB-MAP GUI visua