"""Start the RGB-D target service without looking up or starting Orbbec."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('vision_target_server')
    return LaunchDescription([
        DeclareLaunchArgument('vision_config', default_value=os.path.join(share, 'config', 'vision.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='vision_target_server', executable='vision_server', name='vision_server', output='screen',
             parameters=[LaunchConfiguration('vision_config'), {
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
    ])
