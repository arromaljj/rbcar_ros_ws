#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution


def generate_launch_description():
    # Declare the slam_method argument with default value
    slam_method_arg = DeclareLaunchArgument(
        'slam_method',
        default_value='c_2d',
        description='SLAM method to use: c_2d (Cartographer 2D), c_3d (Cartographer 3D), or st_2d (SLAM Toolbox 2D)'
    )

    # Get the package directory
    pkg_rbcar_model = get_package_share_directory('rbcar_model')
    
    # Define the path to the RViz configuration file
    rviz_config_file = "/root/ros_ws/src/rbcar_model/rviz/slam-1.rviz"
    
    # Include the spawn_rbcar.launch.py to start Gazebo and spawn the robot
    spawn_rbcar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_rbcar_model, 'launch', 'spawn_rbcar.launch.py')])
    )
    
    # Define the slam_method LaunchConfiguration
    slam_method = LaunchConfiguration('slam_method')
    
    # Add log messages to show which SLAM method was chosen
    log_c_2d = LogInfo(
        msg=["SLAM Method: Using Cartographer 2D"],
        condition=IfCondition(PythonExpression(["'", slam_method, "' == 'c_2d'"]))
    )
    
    log_c_3d = LogInfo(
        msg=["SLAM Method: Using Cartographer 3D"],
        condition=IfCondition(PythonExpression(["'", slam_method, "' == 'c_3d'"]))
    )
    
    log_st_2d = LogInfo(
        msg=["SLAM Method: Using SLAM Toolbox 2D"],
        condition=IfCondition(PythonExpression(["'", slam_method, "' == 'st_2d'"]))
    )
    
    # Include Cartographer 2D if slam_method is c_2d
    cartographer_2d = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg_rbcar_model, 'launch', 'cartographer', 'cartographer.launch.py')])
            )
        ],
        condition=IfCondition(PythonExpression(["'", slam_method, "' == 'c_2d'"]))
    )
    
    # Include Cartographer 3D if slam_method is c_3d
    cartographer_3d = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    pkg_rbcar_model, 'launch', 'cartographer', 'cartographer_3d.launch.py')])
            )
        ],
        condition=IfCondition(PythonExpression(["'", slam_method, "' == 'c_3d'"]))
    )
    
    # Include SLAM Toolbox 2D if slam_method is st_2d
    slam_toolbox_2d = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('rbcar_slam'), 'launch', 'slam.launch.py')])
            )
        ],
        condition=IfCondition(PythonExpression(["'", slam_method, "' == 'st_2d'"]))
    )
    
    # Launch RViz with the specified configuration file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Return the LaunchDescription
    return LaunchDescription([
        slam_method_arg,
        log_c_2d,
        log_c_3d,
        log_st_2d,
        spawn_rbcar,
        cartographer_2d,
        cartographer_3d,
        slam_toolbox_2d,
        rviz_node
    ]) 