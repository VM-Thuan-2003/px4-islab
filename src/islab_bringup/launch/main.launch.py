from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    islab_control_launch = os.path.join(
        get_package_share_directory('islab_control'),
        'launch',
        'islab_main_control.launch.py'
    )
    islab_tools_launch = os.path.join(
        get_package_share_directory('islab_tools'),
        'launch',
        'islab_main_tools.launch.py'
    )
    islab_bringup_launch = os.path.join(
        get_package_share_directory('islab_bringup'),
        'launch',
        'islab_main_bringup.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(islab_control_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(islab_tools_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(islab_bringup_launch)
        ),
    ])
    