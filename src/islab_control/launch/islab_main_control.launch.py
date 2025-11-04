#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    islab_control_dir = get_package_share_directory('islab_control')
    islab_control_yaml = os.path.join(islab_control_dir, 'config', 'islab_control.yaml')

    return LaunchDescription([
        Node(
            package='islab_control',
            executable='main',
            name='main',
            output='log',
            parameters=[islab_control_yaml],
        ),
        Node(
            package='islab_control',
            executable='auto_control',
            name='auto_control',
            output='log',
            # parameters=[islab_control_yaml],
        ),
        Node(
            package='islab_control',
            executable='joy_to_px4_manual',
            name='joy_to_px4_manual',
            output='log',
            # parameters=[islab_control_yaml],
        )
    ])