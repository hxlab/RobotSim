from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'frame_id': 'world',
                'device_frame': 'camera',
                'publish_rate': 15.0
            }]
        )
    ])
