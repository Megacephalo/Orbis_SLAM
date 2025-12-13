#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    recording_file_arg = DeclareLaunchArgument(
        'recording_file',
        default_value='',
        description='Path to the .orbis recording file to play back'
    )

    enable_slam_arg = DeclareLaunchArgument(
        'enable_slam',
        default_value='true',
        description='Enable SLAM backend optimization'
    )

    playback_rate_arg = DeclareLaunchArgument(
        'playback_rate',
        default_value='1.0',
        description='Playback speed multiplier (1.0 = real-time)'
    )

    # Define the ZED Playback node
    zed_playback_node = Node(
        package='orbis_slam',
        executable='orbis_slam_playback_node',
        name='zed_playback_node',
        output='screen',
        parameters=[{
            'recording_file': LaunchConfiguration('recording_file'),
            'world_frame': 'map',
            'odom_frame': 'odom',
            'robot_baselink_frame': 'base_link',
            'left_camera_frame': 'left_camera_frame',
            'enable_slam': LaunchConfiguration('enable_slam'),
            'playback_rate': LaunchConfiguration('playback_rate'),
        }]
    )

    # Create and return the launch description
    return LaunchDescription([
        recording_file_arg,
        enable_slam_arg,
        playback_rate_arg,
        zed_playback_node
    ])
