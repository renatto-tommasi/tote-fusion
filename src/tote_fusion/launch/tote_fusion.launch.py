from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    return [Node(package='tote_fusion', executable='tote_fusion_node', output='screen', parameters=[{
        'use_sim_time': True, 'model_file': LaunchConfiguration('model_file'),
        'camera_file': LaunchConfiguration('camera_file'),
        'camera_ids': LaunchConfiguration('camera_ids').perform(context).split(','),
        'validation_profile': LaunchConfiguration('validation_profile'),
        'candidate_validity_s': ParameterValue(LaunchConfiguration('candidate_validity_s'),value_type=float),
        'reliable_sensor_qos': ParameterValue(LaunchConfiguration('reliable_sensor_qos'),value_type=bool),
    }])]


def generate_launch_description():
    share = Path(get_package_share_directory('tote_fusion'))
    return LaunchDescription([
        DeclareLaunchArgument('model_file', default_value=str(share / 'config/tote_model.yaml')),
        DeclareLaunchArgument('camera_file', default_value=str(share / 'config/cameras.yaml')),
        DeclareLaunchArgument('camera_ids', default_value='head', description='Comma-separated camera IDs from the catalog'),
        DeclareLaunchArgument('validation_profile', default_value='head_sim_strict'),
        DeclareLaunchArgument('candidate_validity_s', default_value='0.5'),
        DeclareLaunchArgument('reliable_sensor_qos', default_value='False', choices=['True','False']),
        OpaqueFunction(function=start),
    ])
