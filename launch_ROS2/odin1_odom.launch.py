# USAGE: ros2 launch odin_ros_driver odin1_odom.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('odin_ros_driver')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([package_share, 'launch', 'odin1_ros2.launch.py'])
            ),
            launch_arguments={
                'config_file': PathJoinSubstitution([package_share, 'config', 'control_odom.yaml']),
                'rviz_config': 'auto',
            }.items(),
        ),
    ])
