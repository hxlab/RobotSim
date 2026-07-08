from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gui',
            executable='gui_node',
            name='gui',
            output='screen'
        )
    ])
