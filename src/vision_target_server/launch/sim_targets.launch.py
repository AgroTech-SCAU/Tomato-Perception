"""Launch fixed-world targets for a separately launched simulated robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'calibration_file': ('', str),
        'world_frame': ('base_link', str),
        'tool_frame': ('tool0', str),
        'tcp_frame': ('tip', str),
        'camera_frame': ('camera_color_optical_frame', str),
        'initial_target_distance_m': ('0.30', float),
        'sample_rate_hz': ('30.0', float),
        'confidence': ('0.95', float),
        'default_confidence': ('0.40', float),
        'default_max_frame_age_sec': ('0.50', float),
        'service_name': ('/vision/get_targets', str),
        'dropout_start_request': ('2', int),
        'dropout_requests': ('0', int),
        'ambiguous_start_request': ('2', int),
        'ambiguous_requests': ('0', int),
        'harvest_mode': ('false', bool),
        'harvest_service_name': ('/simulation/harvest', str),
        'grasp_tolerance_m': ('0.006', float),
        'release_tolerance_m': ('0.01', float),
        'retreat_distance_m': ('0.10', float),
        'standoff_m': ('0.08', float),
    }
    declarations = [DeclareLaunchArgument(name, default_value=value) for name, (value, _) in defaults.items()]
    parameters = {name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                  for name, (_, kind) in defaults.items()}
    parameters['use_sim_time'] = False
    # Arbitrary explicit world_target lists can be supplied in a normal ROS parameter file.
    declarations.append(DeclareLaunchArgument('sim_config', default_value=''))
    # An empty params-file path is invalid, so include it only when explicitly provided.
    from launch.actions import OpaqueFunction

    def start(context):
        config = LaunchConfiguration('sim_config').perform(context)
        return [Node(package='vision_target_server', executable='sim_target_server',
                     name='sim_target_server', output='screen',
                     parameters=[parameters, *([config] if config else [])])]

    return LaunchDescription([*declarations, OpaqueFunction(function=start)])
