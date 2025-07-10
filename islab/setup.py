from setuptools import setup
import glob

launch_files = glob.glob('launch/*.py')

setup(
    name='islab',
    version='0.0.0',
    packages=['islab'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/islab']),
        ('share/islab', ['package.xml']),
        ('share/islab/launch', launch_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Albert Benjamin',
    maintainer_email='vmthuan16052003@gmail.com',
    description='islab utilities for ROS2-Ardu UAV',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'visualize = islab.visualize:main',
            'run_pipeline = islab.run_pipeline:main',
            'islab_test = islab.test:main',
        ],
    },
)
