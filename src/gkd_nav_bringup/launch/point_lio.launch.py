"""Official MID-360 CustomMsg/IMU source plus the native Rust Point-LIO adapter."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('gkd_nav_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('start_driver', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution(
            [share, 'config', 'MID360_config.json'])),
        DeclareLaunchArgument('params', default_value=PathJoinSubstitution(
            [share, 'config', 'point_lio.yaml'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [share, 'launch', 'mid360.launch.py'])),
            condition=IfCondition(LaunchConfiguration('start_driver')),
            launch_arguments={
                'config': LaunchConfiguration('config'), 'xfer_format': '1',
                'lidar_topic': LaunchConfiguration('lidar_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
            }.items()),
        Node(
            package='point_lio_bridge', executable='point_lio_node',
            name='point_lio', output='screen',
            parameters=[LaunchConfiguration('params')],
            remappings=[('livox/lidar', LaunchConfiguration('lidar_topic')),
                        ('livox/imu', LaunchConfiguration('imu_topic'))]),
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', PathJoinSubstitution(
                 [share, 'config', 'point_lio.rviz'])]),
    ])
