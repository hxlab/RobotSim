import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, ExecuteProcess,
    RegisterEventHandler, TimerAction
)
from launch.event_handlers import OnProcessExit
from launch import LaunchContext

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

import yaml


def get_custom_world_path():
    return os.path.join(get_package_share_directory('controller'), 'worlds', 'robot_table.sdf')


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def get_robot_description(context: LaunchContext, arm_id, load_gripper, franka_hand, is_gazebo):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)
    is_gazebo_str = context.perform_substitution(is_gazebo)

    franka_xacro_file = os.path.join(
        get_package_share_directory('custom_franka_description'),
        'robots', arm_id_str, arm_id_str + '.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'arm_id': arm_id_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': is_gazebo_str,
            'ee_id': franka_hand_str,
            'gazebo_effort': is_gazebo_str
        }
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_config.toxml()}],
    )

    return [robot_state_publisher]


def generate_launch_description():

    # ========== LAUNCH ARGUMENTS ==========
    load_gripper_launch_argument = DeclareLaunchArgument(
        'load_gripper', default_value='true',
        description='true/false for activating the gripper')
    franka_hand_launch_argument = DeclareLaunchArgument(
        'franka_hand', default_value='franka_hand',
        description='Default value: franka_hand')
    arm_id_launch_argument = DeclareLaunchArgument(
        'arm_id', default_value='fr3',
        description='Available values: fr3, fp3 and fer')
    namespace_launch_argument = DeclareLaunchArgument(
        'namespace', default_value='namespace',
        description='Namespace for the robot.')
    config_launch_argument = DeclareLaunchArgument(
        'controllers_yaml',
        default_value=PathJoinSubstitution(
            [FindPackageShare('controller'), 'config', 'config.yaml']
        ),
        description='Override the default controllers.yaml file.'
    )
    is_gazebo_argument = DeclareLaunchArgument(
        "is_gazebo",
        default_value="true",
        description="Launch in Gazebo simulation mode"
    )

    load_gripper = LaunchConfiguration('load_gripper')
    franka_hand = LaunchConfiguration('franka_hand')
    arm_id = LaunchConfiguration('arm_id')
    namespace = LaunchConfiguration('namespace')
    controllers_yaml = LaunchConfiguration('controllers_yaml')
    is_gazebo = LaunchConfiguration('is_gazebo')

    # ========== ROBOT DESCRIPTION ==========
    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[arm_id, load_gripper, franka_hand, is_gazebo])

    # ========== GAZEBO ==========
    os.environ['GZ_SIM_RESOURCE_PATH'] = (
        os.path.dirname(get_package_share_directory('custom_franka_description'))
    )

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_custom_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': get_custom_world_path() + ' -r'}.items(),
        condition=IfCondition(is_gazebo)
    )

    # Reference to the inner spawn node for event handler chaining
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=[
            '-topic', '/robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '1.0',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0'
        ],
        output='screen',
        condition=IfCondition(is_gazebo)
    )

    clock_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/depth_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
                '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                '/depth_camera/color/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/depth_camera/depth/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            ],
            output='screen',
            condition=IfCondition(is_gazebo)
        )

    # ========== CONTROLLERS ==========
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c", "/controller_manager",
        ],
        parameters=[controllers_yaml],
        output="screen",
    )

    haptic_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "haptic_impedance_controller",
            "-c", "/controller_manager",
            "-t", "controller/HapticImpedanceController",
        ],
        parameters=[controllers_yaml],
        output="screen",
    )

    # ========== LAUNCH DESCRIPTION ==========
    return LaunchDescription([
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        arm_id_launch_argument,
        namespace_launch_argument,
        config_launch_argument,
        is_gazebo_argument,
        # start Gazebo first
        gazebo_custom_world,

        # start robot state publisher immediatel
        robot_state_publisher,

        # spawn robot after Gazebo has loaded
        spawn_node,  # needed for OnProcessExit event handler below

        clock_node,

        # load controllers after spawn completes
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_node,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[haptic_controller_spawner],
            )
        ),
    ])