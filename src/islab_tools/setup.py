from setuptools import find_packages, setup

package_name = 'islab_tools'

import glob
launch_files = glob.glob('launch/*.py')
config_files = glob.glob('config/*.yaml')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test','model']),
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
            'gui = islab_tools.gui:main',
            'bridge = islab_tools.bridge:main',
            'drop_ball = islab_tools.drop_ball:main',
            'score = islab_tools.score:main',
        ],
    },
)
