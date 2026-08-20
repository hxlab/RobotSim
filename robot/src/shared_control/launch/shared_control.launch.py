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
            package='shared_control',
            executable='shared_control',
            name='shared_control',
            output='screen'
        )
    ])
