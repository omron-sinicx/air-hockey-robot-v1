from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'air_hockey_robot_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # py_modules=[
    #     'air_hockey_robot_py/arm_motion_planner.py'
    # ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='osx',
    maintainer_email='kazutoshi.tanaka@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_motion_planner = air_hockey_robot_py.arm_motion_planner:main',
            'motor_communication = air_hockey_robot_py.motor_communication:main',
            'py_camera_driver = air_hockey_robot_py.py_camera_driver:main'
        ],
    },
)
