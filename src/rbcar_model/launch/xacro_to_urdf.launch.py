#!/usr/bin/env python3

import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext,LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command


def generate_launch_description():
    # Get timestamp at launch file parsing time
    timestamp = str(int(time.time()))
    
    # Declare arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='rbcar2.urdf.xacro',
        description='Path to the xacro model file, relative to robots/ directory'
    )

    pkg_rbcar_model = get_package_share_directory('rbcar_model')
    
    # Input xacro file path
    xacro_file = PathJoinSubstitution([
        pkg_rbcar_model,
        'robots',
        LaunchConfiguration('model_path')
    ])

    # Get model path string using Command
    context = LaunchContext()
    model_path_arg.execute(context)
    model_path_str = LaunchConfiguration('model_path').perform(context)
    model_base_name = model_path_str.split(".")[0]
    model_base_name_log = LogInfo(msg=[model_base_name])
    # Output file path (just timestamp.urdf)
    output_path = os.path.join(os.getcwd(), 'src', 'rbcar_model', 'out', f"{model_base_name}_{timestamp}.urdf")

    # Create xacro command using Command substitution
    generate_urdf_cmd = ExecuteProcess(
        cmd=[
            'xacro',
            xacro_file,
            'prefix:=robot_',
            '-o',
            output_path
        ]
    )

    # Log files
    log_xacro_file = LogInfo(msg=['Input XACRO file: ', xacro_file])
    log_output_file = LogInfo(msg=['Output URDF file: ', output_path])

    return LaunchDescription([
        model_base_name_log,
        model_path_arg,
        log_xacro_file,
        log_output_file,
        generate_urdf_cmd
    ])