#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # islab_tools_dir = get_package_share_directory('islab_tools')
    # islab_gui_yaml = os.path.join(islab_tools_dir, 'config', 'islab_gui.yaml')
    # islab_bridge_yaml = os.path.join(islab_tools_dir, 'config', 'islab_bridge.yaml')
    # islab_ball_yaml = os.path.join(islab_tools_dir, 'config', 'islab_drop_ball.yaml')

    return LaunchDescription([
        Node(
            package='islab_tools',
            executable='gui',
            name='gui',
            output='log',
            # parameters=[islab_gui_yaml],
        ),
        Node(
            package='islab_tools',
            executable='bridge',
            name='bridge',
            output='log',
            # parameters=[islab_bridge_yaml],
        ),
        Node(
            package='islab_tools',
            executable='drop_ball',
            name='drop_ball',
            output='log',
            # parameters=[islab_ball_yaml],
        ),
        Node(
            package='islab_tools',
            executable='score',
            name='score',
            output='log',
            # parameters=[islab_ball_yaml],
        ),
        Node(
            package='islab_tools',
            executable='read_status_gazebo',
            name='read_status_gazebo',
            output='log',
            # parameters=[islab_ball_yaml],
        ),
        # Node(
        #     package='islab_tools',
        #     executable='keyboard_joy',
        #     name='keyboard_joy',
        #     output='screen',
        #     # parameters=[islab_ball_yaml],
        # ),
    ])