from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    is_gazebo_argument = DeclareLaunchArgument(
            "is_gazebo",
            default_value="true",
            description="Launch in Gazebo simulation mode"
        )

    return LaunchDescription([
        is_gazebo_argument,
        
        Node(
            package='gui',
            executable='gui_app',
            name='gui',
            output='screen',
            additional_env={
                '__NV_PRIME_RENDER_OFFLOAD': '1',
                '__GLX_VENDOR_LIBRARY_NAME': 'nvidia'
            }
        )
    ])
