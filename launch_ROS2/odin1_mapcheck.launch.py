# USAGE: ros2 launch odin_ros_driver odin1_mapcheck.launch.py [map_file:=/path/to/map.bin] [pcd_map_file:=/path/to/map.pcd] [rviz_config:=auto]
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def resolve_rviz_config(package_share_dir, rviz_config):
    if rviz_config != 'auto':
        return rviz_config

    return os.path.join(package_share_dir, 'config', 'odin_relocalization_mapcheck.rviz')


def normalize_path(path):
    return os.path.abspath(os.path.expanduser(path))


def resolve_pcd_map_file(cli_pcd_map_file):
    if not cli_pcd_map_file:
        return '', ''

    pcd_map_file = normalize_path(cli_pcd_map_file)
    if not os.path.isfile(pcd_map_file):
        return pcd_map_file, f'CLI pcd_map_file does not exist: {pcd_map_file}'

    if not pcd_map_file.lower().endswith('.pcd'):
        return pcd_map_file, f'CLI pcd_map_file is not a .pcd file: {pcd_map_file}'

    return pcd_map_file, ''


def abort_actions(reason):
    return [
        LogInfo(msg=f'[ERROR] {reason}'),
        EmitEvent(event=Shutdown(reason=reason)),
    ]


def launch_setup(context, *args, **kwargs):
    package_share_dir = get_package_share_directory('odin_ros_driver')
    map_file = LaunchConfiguration('map_file').perform(context).strip()
    cli_pcd_map_file = LaunchConfiguration('pcd_map_file').perform(context).strip()
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context)
    rviz_config = resolve_rviz_config(
        package_share_dir,
        LaunchConfiguration('rviz_config').perform(context).strip(),
    )
    pcd_map_file, error = resolve_pcd_map_file(cli_pcd_map_file)
    if error:
        return abort_actions(error)

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('odin_ros_driver'),
                    'launch',
                    'odin1_relocalization.launch.py',
                ])
            ),
            launch_arguments={
                'map_file': map_file,
                'rviz_config': rviz_config,
                'launch_rviz': launch_rviz,
            }.items(),
        ),
    ]

    if pcd_map_file:
        actions.append(
            Node(
                package='odin_ros_driver',
                executable='static_pcd_map_publisher',
                name='static_pcd_map_publisher',
                output='screen',
                parameters=[{
                    'pcd_map_file': pcd_map_file,
                    'topic_name': '/odin1/map_cloud',
                    'frame_id': 'odin1_map',
                }],
            )
        )

    return actions


def generate_launch_description():
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Optional relocalization map .bin path override.',
    )

    pcd_map_file_arg = DeclareLaunchArgument(
        'pcd_map_file',
        default_value='',
        description='Optional reference map .pcd path for static_pcd_map_publisher.',
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='auto',
        description=(
            'Path to RViz2 config file, or auto for '
            'config/odin_relocalization_mapcheck.rviz.'
        ),
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='是否随该 launch 启动 RViz2。',
    )

    return LaunchDescription([
        map_file_arg,
        pcd_map_file_arg,
        rviz_config_arg,
        launch_rviz_arg,
        OpaqueFunction(function=launch_setup),
    ])
