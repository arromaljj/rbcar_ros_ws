from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
    rbcar_nav2_dir = get_package_share_directory('rbcar_nav2')

    # Get the path to the nav2 params file
    nav2_params_path = os.path.join(rbcar_nav2_dir, 'param', 'nav2_params.yaml')

    # Get the path to Nav2's default rviz config
    nav2_default_view = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # Launch Navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_path
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', nav2_default_view],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        navigation_launch,
        rviz_node
    ])
