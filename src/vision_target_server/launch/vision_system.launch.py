import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    vision_share = get_package_share_directory('vision_target_server')
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('orbbec_camera'), 'launch', 'gemini_330_series.launch.py'])),
        condition=IfCondition(LaunchConfiguration('start_camera')),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'serial_number': LaunchConfiguration('camera_serial_number'),
            'enable_color': 'true', 'enable_depth': 'true',
            'color_width': '1280', 'color_height': '800', 'color_fps': '30',
            'depth_width': '1280', 'depth_height': '800', 'depth_fps': '30',
            'depth_registration': 'true', 'align_mode': 'SW', 'align_target_stream': 'COLOR',
            'enable_frame_sync': 'true', 'depth_precision': '1mm',
            'enable_point_cloud': 'true', 'enable_colored_point_cloud': 'false',
        }.items(),
    )
    # The parameter file remains the authority for device/model and input topics
    server = Node(
        package='vision_target_server', executable='vision_server', name='vision_server', output='screen',
        parameters=[LaunchConfiguration('vision_config'), {
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('camera_serial_number', default_value=''),
        DeclareLaunchArgument('vision_config', default_value=os.path.join(vision_share, 'config', 'vision.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        camera, server,
    ])
