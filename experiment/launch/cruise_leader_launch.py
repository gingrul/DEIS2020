from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='deis_py_dev',
            namespace='robot',
            executable='key_subscriber',
            name='key_subscriber',
            remappings=[
            ('/EM','/EM1'),
            ('/platooning','/platooning1'),]
        ),
        Node(
            package='deis_py_dev',
            #namespace='robot',
            executable='key_publisher',
            name='key_publisher'
        )
        
    ])
