#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    islab_bringup_dir = get_package_share_directory('islab_bringup')
    islab_bringup_yaml = os.path.join(islab_bringup_dir, 'config', 'islab_bringup.yaml')

    return LaunchDescription([
        Node(
            package='islab_bringup',
            executable='main',
            name='main',
            output='log',
            parameters=[islab_bringup_yaml],
        ),
    ])