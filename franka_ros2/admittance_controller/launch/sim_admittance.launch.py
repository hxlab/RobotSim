# Copyright (c) 2024 Franka Robotics GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, ExecuteProcess,
    RegisterEventHandler, TimerAction
)
from launch.event_handlers import OnProcessExit
from launch import LaunchContext

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def get_custom_world_path():
    return os.path.join(get_package_share_directory('admittance_controller'), 'worlds', 'robot_table.sdf')


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def get_robot_description(context: LaunchContext, arm_id, load_gripper, franka_hand):
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', arm_id_str, arm_id_str + '.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'arm_id': arm_id_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str,
            'gazebo_effort': 'true'
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
        'namespace', default_value='',
        description='Namespace for the robot.')

    load_gripper = LaunchConfiguration('load_gripper')
    franka_hand = LaunchConfiguration('franka_hand')
    arm_id = LaunchConfiguration('arm_id')
    namespace = LaunchConfiguration('namespace')

    # ========== ROBOT DESCRIPTION ==========
    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[arm_id, load_gripper, franka_hand])

    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory('franka_description'),
            'robots/fr3/fr3.urdf.xacro'),
        mappings={
            'arm_id': 'fr3', 'hand': 'true',
            'ros2_control': 'true', 'gazebo': 'true', 'ee_id': 'franka_hand', 'gazebo_effort': 'true'
        }
    )
    robot_description_semantic_config = xacro.process_file(
        os.path.join(
            get_package_share_directory('franka_description'),
            'robots/fr3/fr3.srdf.xacro'),
        mappings={'hand': 'true', 'ee_id': 'franka_hand'}
    )

    # ========== MOVEIT ==========
    kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')
    ompl_planning_yaml = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml')
    moveit_simple_controllers_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/fr3_controllers.yaml')

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters':
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/ResolveConstraintFrames '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        namespace=namespace,
        output='screen',
        parameters=[
            {'robot_description': robot_description_config.toxml()},
            {'robot_description_semantic': robot_description_semantic_config.toxml()},
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=namespace,
        arguments=['-d', os.path.join(
            get_package_share_directory('franka_fr3_moveit_config'), 'rviz', 'moveit.rviz')],
        parameters=[
            {'robot_description': robot_description_config.toxml()},
            {'robot_description_semantic': robot_description_semantic_config.toxml()},
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
        output='screen',
    )

    # ========== GAZEBO ==========
    os.environ['GZ_SIM_RESOURCE_PATH'] = (
        os.path.dirname(get_package_share_directory('franka_description'))
    )

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_custom_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': get_custom_world_path() + ' -r'}.items(),
    )

    # Spawn Franka — delayed to give Gazebo time to fully load the world
    spawn = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                namespace=namespace,
                arguments=[
                    '-topic', '/robot_description',
                    '-x', '0.0', '-y', '0.0', '-z', '1.0',
                    '-R', '0.0', '-P', '0.0', '-Y', '0.0'
                ],
                output='screen',
            )
        ]
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
    )

    # ========== CONTROLLERS ==========
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_impedance_example_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_impedance_example_controller'],
        output='screen'
    )

    # ========== OTHER NODES ==========
    admittance_control_node = Node(
        package='admittance_controller',
        executable='admittance_control_node',
        name='admittance_control_node',
        namespace=namespace,
        output='screen',
        parameters=[
            {'robot_description': robot_description_config.toxml()},
            {'robot_description_semantic': robot_description_semantic_config.toxml()},
            kinematics_yaml,
            {'force_scale_x': 0.1},
            {'force_scale_y': 0.1},
            {'force_scale_z': 0.1},
            {'max_force_output': 2.0}
        ]
    )


    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[{'source_list': ['/joint_states'], 'rate': 30}],
    )

    # ========== LAUNCH DESCRIPTION ==========
    return LaunchDescription([
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        arm_id_launch_argument,
        namespace_launch_argument,

        # 1. Start Gazebo first
        gazebo_custom_world,

        # 2. Start robot state publisher immediately (no Gazebo dependency)
        robot_state_publisher,

        # 3. Delay RViz and MoveIt slightly so RSP is ready
        TimerAction(period=3.0, actions=[rviz_node]),
        TimerAction(period=3.0, actions=[move_group_node]),

        # 4. Spawn robot after Gazebo has loaded (5s delay)
        spawn,
        spawn_node,  # needed for OnProcessExit event handler below

        # 6. Load controllers after spawn completes
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_node,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_impedance_example_controller],
            )
        ),

        admittance_control_node,
        joint_state_publisher_node,
    ])