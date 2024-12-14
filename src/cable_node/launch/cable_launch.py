import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            remappings=[
                ('/joy', '/joy2')
            ],
            arguments=['--dev', '/dev/input/js1']
        ),
        Node(
            package='cable_node',
            executable='cable_node',
            name='cable_node',
        )
    ])
