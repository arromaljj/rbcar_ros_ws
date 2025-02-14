#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, LogInfo, DeclareLaunchArgument, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue






def generate_launch_description():
    # Get the package directory
    pkg_rbcar_model = get_package_share_directory('rbcar_model')

    # Log the package directory for debugging
    log_pkg_path = LogInfo(msg=f'rbcar_model package path: {pkg_rbcar_model}')

    resource_path = pkg_rbcar_model + ':' + os.environ.get('GAZEBO_RESOURCE_PATH', '')
    log_resource_path_log = LogInfo(msg=f'Resource path: {resource_path}')
    # Set Gazebo resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=pkg_rbcar_model + ':' + os.environ.get('GAZEBO_RESOURCE_PATH', '')
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'verbose': 'true',
            'debug': 'true'
        }.items()
    )

    # URDF file path (using pre-generated URDF)
    urdf_file = os.path.join(pkg_rbcar_model, 'out', 'rbcar2.urdf')
    
    # Log URDF file path for debugging
    log_urdf_path = LogInfo(msg=f'URDF file path: {urdf_file}')
    
    # Read the URDF file content directly
    with open(urdf_file, 'r') as file:
        robot_description = ParameterValue(file.read(), value_type=str)

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'publish_frequency': 30.0
        }]
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'rate': 30,
            'use_sim_time': True
        }]
    )

    # Spawn the robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_rbcar',
        output='screen',
        arguments=['-topic', '/robot_description',
                  '-entity', 'rbcar',
                  '-x', '0.0',
                  '-y', '0.0',
                  '-z', '0.5'],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        log_resource_path_log,
        gazebo_resource_path,
        # log_pkg_path,
        # log_urdf_path,
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot
    ])
