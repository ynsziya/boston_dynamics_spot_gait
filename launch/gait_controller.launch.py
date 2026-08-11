#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('robot_dog_gait')
    gait_params = os.path.join(pkg, 'config', 'gait_params.yaml')
    robot_dims = os.path.join(pkg, 'config', 'robot_dimensions.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gait_params', default_value=gait_params),
        DeclareLaunchArgument('robot_dimensions', default_value=robot_dims),

        Node(
            package='robot_dog_gait',
            executable='robot_dog_controller',
            name='robot_dog_controller',
            output='screen',
            parameters=[
                LaunchConfiguration('gait_params'),
                LaunchConfiguration('robot_dimensions'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('cmd_vel', '/cmd_vel'),
            ],
        ),
    ])
