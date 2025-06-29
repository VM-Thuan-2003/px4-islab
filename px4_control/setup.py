from setuptools import setup
import glob

launch_files = glob.glob('launch/*.py')

setup(
    name='px4_control',
    version='0.0.0',
    packages=['px4_control'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/px4_control']),
        ('share/px4_control', ['package.xml']),
        ('share/px4_control/launch', launch_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Albert Benjamin',
    maintainer_email='vmthuan16052003@gmail.com',
    description='px4_control utilities for ROS2-Ardu UAV',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'manual_control = px4_control.manual_control:main',
            'velocity_control = px4_control.velocity_control:main',
            'auto_control = px4_control.auto_control:main',
        ],
    },
)
