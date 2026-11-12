"""Project MID-360 configuration with the official Livox RViz display."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    project_share = get_package_share_directory('gkd_nav_bringup')
    driver_share = get_package_share_directory('livox_ros_driver2')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=PathJoinSubstitution(
                [project_share, 'config', 'MID360_config.json'])),
        DeclareLaunchArgument('publish_freq', default_value='10.0'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [project_share, 'launch', 'mid360.launch.py'])),
            launch_arguments={
                'config': LaunchConfiguration('config'),
                'publish_freq': LaunchConfiguration('publish_freq'),
                'xfer_format': '0',
                'lidar_topic': LaunchConfiguration('lidar_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
            }.items()),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', PathJoinSubstitution(
                [driver_share, 'config', 'display_point_cloud_ROS2.rviz'])],
            remappings=[('/livox/lidar', LaunchConfiguration('lidar_topic'))]),
    ])
