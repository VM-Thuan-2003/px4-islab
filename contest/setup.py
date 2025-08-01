from setuptools import find_packages, setup
import glob

package_name = 'contest'
launch_files = glob.glob('launch/*.py')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test','model']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', launch_files),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='albert-laptop',
    maintainer_email='vmthuan16052003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pipeline = contest.pipeline:main',
            'drop_balls = contest.drop_balls:main',
            'gui = contest.gui:main',
            'test_contact = contest.test_contact:main',
        ],
    },
)
