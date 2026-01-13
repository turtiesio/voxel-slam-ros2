#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_share_dir = get_package_share_directory('voxel_slam')

    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to launch RViz2'
    )

    config_file = os.path.join(package_share_dir, 'config', 'avia_fly.yaml')
    rviz_config = os.path.join(package_share_dir, 'rviz_cfg', 'back.rviz')

    # VoxelSLAM node
    voxelslam_node = Node(
        package='voxel_slam',
        executable='voxelslam',
        name='voxelslam',
        output='screen',
        parameters=[config_file, {'finish': False}],
    )

    # RViz2 node (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        voxelslam_node,
        rviz_node,
    ])
