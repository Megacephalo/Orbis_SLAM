#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_orbis_slam = get_package_share_directory('orbis_slam')

    zed2i_model_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed2i_ros2_description'),
                'launch',
                'zed_camera.launch.py'
            ])
        ])
    )
    
    
    # Define the ORB-SLAM3 node
    orbis_slam_node = Node(
        package='orbis_slam',
        executable='orbis_slam',
        name='orbis_slam_node',
        output='screen',
        parameters=[{
            'odom_frame'            : 'odom',
            'robot_baselink_frame'  : 'zed2i_camera_link',
            'left_camera_frame'     : 'zed2i_left_camera_optical_frame',
            'enable_slam'           : True,
        }]
    )
    
    # Create and return the launch description
    return LaunchDescription([
        zed2i_model_bringup,
        orbis_slam_node
    ])