from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

model = 'islab_contest'
world = 'contest_1'

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('px4_control'),
                    'launch',
                    'merge_control.launch.py'
                ])
            )
        ),
        Node(
            package='contest',
            executable='pipeline',
            name='pipeline',
            output='screen',
            arguments=[model, world],
        ),
        Node(
            package='contest',
            executable='gui',
            name='gui',
            output='screen',
        ),
        Node(
            package='contest',
            executable='drop_balls',
            name='drop_balls',
            output='screen',
        )
    ])
