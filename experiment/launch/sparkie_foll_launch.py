from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='deis_py_dev',
            namespace='robot',
            executable='sparkie',
            name='sparkie'
        ),
        Node(
            package='deis_py_dev',
            namespace='robot',
            executable='follower',
            name='follower'
        )
    ])
