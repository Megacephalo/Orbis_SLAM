#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription



def generate_launch_description():
    # Declare launch arguments
    recording_file_arg = DeclareLaunchArgument(
        'recording_file',
        default_value=str(
            Path.home()
            / 'Documents'
            / 'Datasts'
            / 'orbis_slam_samples'
            / 'recording_20251228_170853.orbis'
        ),
        description='Path to the .orbis recording file to play back'
    )

    playback_rate_arg = DeclareLaunchArgument(
        'playback_rate',
        default_value='1.0',
        description='Playback speed multiplier (1.0 = real-time)'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    enable_loop_arg = DeclareLaunchArgument(
        'enable_loop',
        default_value='true',
        description='Enable looping playback (restart from beginning when finished)'
    )

    zed_2i_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('zed2i_ros2_description'),
                'launch',
                'zed_camera.launch.py'
            ])
        ),
        launch_arguments={
            'enable_rviz': 'false',
        }.items()
    )

    # Define the ZED Playback node
    # Frame names must match the zed2i URDF: zed2i_base_link is the camera root frame
    zed_playback_node = Node(
        package='orbis_slam',
        executable='orbis_slam_playback_node',
        name='zed_playback_node',
        output='screen',
        parameters=[{
            'recording_file': LaunchConfiguration('recording_file'),
            'playback_rate': LaunchConfiguration('playback_rate'),
            'enable_loop': LaunchConfiguration('enable_loop'),
            'world_frame': 'map',
            'odom_frame': 'odom',
            'robot_baselink_frame': 'zed2i_base_link',  # Connect to camera URDF root
            'left_camera_frame': 'zed2i_left_camera_optical_frame',
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    # Create and return the launch description
    return LaunchDescription([
        recording_file_arg,
        playback_rate_arg,
        launch_rviz_arg,
        enable_loop_arg,
        zed_2i_description,
        zed_playback_node,
        rviz_node
    ])
