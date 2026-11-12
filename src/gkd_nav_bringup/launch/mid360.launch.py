"""Start the official MID-360 driver with project-owned network configuration."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    project_share = get_package_share_directory('gkd_nav_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=PathJoinSubstitution(
                [project_share, 'config', 'MID360_config.json']),
            description='MID-360 device and host network configuration'),
        DeclareLaunchArgument(
            'xfer_format', default_value='1',
            choices=['0', '1'], description='0: PointCloud2; 1: Livox CustomMsg'),
        DeclareLaunchArgument('publish_freq', default_value='10.0'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        Node(
            package='livox_ros_driver2', executable='livox_ros_driver2_node',
            name='livox_lidar_publisher', output='screen', parameters=[{
                'xfer_format': ParameterValue(
                    LaunchConfiguration('xfer_format'), value_type=int),
                'multi_topic': 0, 'data_src': 0, 'output_data_type': 0,
                'publish_freq': ParameterValue(
                    LaunchConfiguration('publish_freq'), value_type=float),
                'frame_id': 'livox_frame',
                'user_config_path': LaunchConfiguration('config'),
                'lvx_file_path': '/tmp/unused.lvx',
                'cmdline_input_bd_code': 'livox0000000001',
            }], remappings=[
                ('livox/lidar', LaunchConfiguration('lidar_topic')),
                ('livox/imu', LaunchConfiguration('imu_topic')),
            ]),
    ])
