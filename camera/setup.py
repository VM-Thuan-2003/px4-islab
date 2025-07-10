from setuptools import setup

setup(
    name='camera',
    version='0.0.0',
    packages=['camera'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/camera']),
        ('share/camera', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Albert Benjamin',
    maintainer_email='vmthuan16052003@gmail.com',
    description='camera utilities for ROS2-Ardu UAV',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pub_camera = camera.pub_camera:main',
            'read_camera = camera.read_camera:main',
            'processing_camera = camera.processing_camera:main',
        ],
    },
)