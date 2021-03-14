import os
from glob import glob
from setuptools import setup

package_name = 'deis_py_dev'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CG',
    maintainer_email='christogeorge@live.in',
    description='DEV - ROS2 nodes for DEIS Robo',
    license='Copyright NO one is fucking allowed to touch it',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #these nodes are from the package deis_py_dev
            #you run these nodes
            'main_node = deis_py_dev.main:main',
            'sparkie = deis_py_dev.sparkie:main',
            'teleop = deis_py_dev.teleop:main',
            'gps = deis_py_dev.gps:main',
            'odometer = deis_py_dev.odometer:main',
            'imu = deis_py_dev.imu:main',
            'teleop_C_node = deis_py_dev.teleop_C:main',
            'follower = deis_py_dev.follower:main',
            'drone = deis_py_dev.drone:main',
            'roof = deis_py_dev.roof:main',
            'teleop_cmd = deis_py_dev.teleop_cmd:main',
            'emergency = deis_py_dev.emergency:main',
            'navigator = deis_py_dev.navigator:main',
            'scanner = deis_py_dev.scanner:main',
            'triangle = deis_py_dev.triangleS:main',
            'notifier = deis_py_dev.notifier:main',
            'rescuer = deis_py_dev.rescuer:main',
            'key_publisher = deis_py_dev.KeyPublisher:main',
            'key_subscriber = deis_py_dev.KeySubscriber:main'
        ],
    },
)
