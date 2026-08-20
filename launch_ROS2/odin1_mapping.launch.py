# USAGE: ros2 launch odin_ros_driver odin1_mapping.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('odin_ros_driver')

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='是否随该 launch 启动 RViz2',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([package_share, 'launch', 'odin1_ros2.launch.py'])
            ),
            launch_arguments={
                'config_file': PathJoinSubstitution([package_share, 'config', 'control_mapping.yaml']),
                'rviz_config': 'auto',
                'launch_rviz': LaunchConfiguration('launch_rviz'),
            }.items(),
        ),
    ])
