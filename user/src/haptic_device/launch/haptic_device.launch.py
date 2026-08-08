from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='haptic_device',
            executable='haptic_device_node',
            name='haptic_device',
            output='screen',
            parameters=[{
                'frame_id': 'world',
                'device_frame': 'haptic_device',
                'publish_rate': 100.0,
                'publish_tf': True
            }]
        )
    ])
