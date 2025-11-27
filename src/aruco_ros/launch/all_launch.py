from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # ===== RealSense =====
    realsense_launch_file = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file)
    )

    # ===== ArUco Detector Node =====
    aruco_detector = Node(
        package='aruco_ros',
        executable='aruco_detector_node',
        output='screen'
    )

    # ===== Robot Detector Node =====
    robot_detector = Node(
        package='aruco_ros',
        executable='robot_detector_node',
        output='screen'
    )

    # ===== RViz2 =====
    rviz = ExecuteProcess(
        cmd=['rviz2'],
        output='screen'
    )

    return LaunchDescription([
        realsense,
        aruco_detector,
        robot_detector,
        rviz
    ])
