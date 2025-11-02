from setuptools import find_packages, setup
import glob

package_name = 'islab_control'

launch_files = glob.glob('launch/*.py')
config_files = glob.glob('config/*.yaml')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+package_name+'/launch', launch_files),
        ('share/'+package_name+'/config', config_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='albert-laptop',
    maintainer_email='vmthuan16052003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'main = islab_control.main:main',
            'auto_control = islab_control.auto_control:main',
            'joy_to_px4_manual = islab_control.manual_control:main',
        ],
    },
)
