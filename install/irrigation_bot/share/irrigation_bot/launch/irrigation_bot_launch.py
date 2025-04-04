from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='irrigation_bot',
            executable='irrigation_bot',
            name='irrigation_bot',
            output='screen'
        )
    ])
