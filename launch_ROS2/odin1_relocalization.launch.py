# USAGE: ros2 launch odin_ros_driver odin1_relocalization.launch.py [map_file:=/path/to/map.bin]
import os

import yaml
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
from launch.substitutions import LaunchConfiguration


def package_source_dir(package_dir):
    candidates = []

    for prefix in os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep):
        if not prefix:
            continue
        install_pos = prefix.find('/install')
        if install_pos >= 0:
            candidates.append(os.path.join(prefix[:install_pos], 'src', 'odin_ros_driver'))

    install_pos = package_dir.find('/install')
    if install_pos >= 0:
        candidates.append(os.path.join(package_dir[:install_pos], 'src', 'odin_ros_driver'))

    for candidate in candidates:
        if os.path.isdir(candidate):
            return candidate

    return package_dir


def load_yaml(config_file):
    with open(config_file, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def yaml_relocalization_map(config_file):
    params = load_yaml(config_file)
    register_keys = params.get('register_keys', {})
    if not isinstance(register_keys, dict):
        return ''

    map_file = register_keys.get('relocalization_map_abs_path', '')
    return str(map_file).strip() if map_file else ''


def latest_bin_map(package_dir):
    map_dir = os.path.join(package_source_dir(package_dir), 'map')
    if not os.path.isdir(map_dir):
        return ''

    bin_files = []
    for root, _, files in os.walk(map_dir):
        for name in files:
            if name.lower().endswith('.bin'):
                path = os.path.join(root, name)
                if os.path.isfile(path):
                    bin_files.append(path)

    if not bin_files:
        return ''

    return max(bin_files, key=os.path.getmtime)


def normalize_path(path):
    return os.path.abspath(os.path.expanduser(path))


def resolve_map_file(cli_map_file, config_file, package_dir):
    if cli_map_file:
        map_file = normalize_path(cli_map_file)
        if not os.path.isfile(map_file):
            return 'cli', map_file, f'CLI map_file does not exist: {map_file}'
        return 'cli', map_file, ''

    yaml_map_file = yaml_relocalization_map(config_file)
    if yaml_map_file:
        map_file = normalize_path(yaml_map_file)
        if not os.path.isfile(map_file):
            return 'yaml', map_file, (
                f'YAML relocalization_map_abs_path does not exist: {map_file}'
            )
        return 'yaml', map_file, ''

    map_file = latest_bin_map(package_dir)
    if not map_file:
        return (
            'auto latest',
            '',
            'No .bin relocalization map found under src/odin_ros_driver/map',
        )

    return 'auto latest', map_file, ''


def abort_actions(reason):
    return [
        LogInfo(msg=f'[ERROR] {reason}'),
        EmitEvent(event=Shutdown(reason=reason)),
    ]


def launch_setup(context, *args, **kwargs):
    package_dir = get_package_share_directory('odin_ros_driver')
    config_file = os.path.join(package_dir, 'config', 'control_relocalization.yaml')
    cli_map_file = LaunchConfiguration('map_file').perform(context).strip()
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context)

    source, map_file, error = resolve_map_file(cli_map_file, config_file, package_dir)
    if error:
        return abort_actions(error)

    return [
        LogInfo(msg=f'Selected relocalization map ({source}): {map_file}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_dir, 'launch', 'odin1_ros2.launch.py')
            ),
            launch_arguments={
                'config_file': config_file,
                'rviz_config': rviz_config,
                'map_file': map_file,
                'launch_rviz': launch_rviz,
            }.items(),
        ),
    ]


def generate_launch_description():
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description=(
            'Optional relocalization map .bin path. '
            'Empty uses YAML path, then newest map/*/*.bin.'
        ),
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='auto',
        description='Path to RViz2 config file, or auto for normal relocalization RViz.',
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='是否随该 launch 启动 RViz2。',
    )

    return LaunchDescription([
        map_file_arg,
        rviz_config_arg,
        launch_rviz_arg,
        OpaqueFunction(function=launch_setup),
    ])
