from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_control',
            executable='manual_control',
            name='manual_control',
            output='screen',
            prefix='gnome-terminal --geometry=65x24 --',
        ),
        Node(
            package='px4_control',
            executable='auto_control',
            name='auto_control',
            output='screen',
            prefix='gnome-terminal --geometry=65x24 --',
        ),
        Node(
            package='px4_control',
            executable='velocity_control',
            name='velocity_control',
            output='screen'
        ),
    ])
