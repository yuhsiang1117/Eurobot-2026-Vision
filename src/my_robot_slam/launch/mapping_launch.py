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