#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_orbis_slam = get_package_share_directory('orbis_slam')

    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='Enable RViz visualization'
    )
    rviz_config_file = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([
            pkg_orbis_slam,
            'launch',
            'sensor_only_view.rviz'
        ]),
        description='Full path to the RViz config file to use'
    )

    zed2i_model_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed2i_ros2_description'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'enable_rviz': LaunchConfiguration('enable_rviz')
        }.items()
    )
    
    
    # Define the ORB-SLAM3 node
    orbis_slam_node = Node(
        package='orbis_slam',
        executable='orbis_slam_node',
        name='orbis_slam_node',
        output='screen',
        parameters=[{
            'world_frame'           : 'map',
            'odom_frame'            : 'odom',
            'robot_baselink_frame'  : 'zed2i_camera_link',
            'cam_center_frame'      : 'zed_2i_camera_center',
            'left_camera_frame'     : 'zed2i_left_camera_optical_frame',
            'enable_slam'           : True,
            'cam_pointcloud_topic'  : 'zed/point_cloud'
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='Orbis_SLAM_rviz2',
        output='screen',
        arguments=["-d", LaunchConfiguration('rviz_config_file')],
    )
    
    # Create and return the launch description
    return LaunchDescription([
        enable_rviz_arg,
        rviz_config_file,
        zed2i_model_bringup,
        orbis_slam_node,
        rviz_node
    ])