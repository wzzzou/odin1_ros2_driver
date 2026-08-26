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
    try:
        params = load_yaml(config_file)
    except (OSError, yaml.YAMLError) as exc:
        return '', f'Failed to read relocalization config {config_file}: {exc}'

    register_keys = params.get('register_keys', {})
    if not isinstance(register_keys, dict):
        return '', f'Invalid register_keys section in {config_file}'

    map_file = register_keys.get('relocalization_map_abs_path', '')
    if map_file in (None, ''):
        return '', ''
    if not isinstance(map_file, str):
        return '', (
            f'register_keys.relocalization_map_abs_path must be a string in {config_file}'
        )
    return map_file.strip(), ''


def map_search_directories(package_dir, map_dir):
    # CLI map_dir is the strictest operator selection. If it is invalid, do
    # not silently replace it with an environment or package-local directory.
    if map_dir:
        candidates = [normalize_path(map_dir)]
    else:
        env_map_dir = os.environ.get('ODIN_MAP_DIR', '').strip()
        if env_map_dir:
            candidates = [normalize_path(env_map_dir)]
        else:
            # Keep the source-tree fallback for this workspace, where generated
            # maps are intentionally ignored and stored outside install artifacts.
            candidates = [
                os.path.join(package_dir, 'map'),
                os.path.join(package_source_dir(package_dir), 'map'),
            ]

    unique = []
    for candidate in candidates:
        if candidate and candidate not in unique and os.path.isdir(candidate):
            unique.append(candidate)
    return unique


def configured_map_dir(map_dir):
    if map_dir:
        return 'CLI map_dir', normalize_path(map_dir)

    env_map_dir = os.environ.get('ODIN_MAP_DIR', '').strip()
    if env_map_dir:
        return 'ODIN_MAP_DIR', normalize_path(env_map_dir)

    return '', ''


def latest_bin_map(package_dir, map_dir):
    # Explicit map_dir and ODIN_MAP_DIR are ordered overrides. Do not merge
    # their contents with fallback directories, otherwise an unrelated newer
    # map can silently win over the operator-selected directory.
    for search_dir in map_search_directories(package_dir, map_dir):
        bin_files = []
        for root, _, files in os.walk(search_dir):
            for name in files:
                if name.lower().endswith('.bin'):
                    path = os.path.join(root, name)
                    if os.path.isfile(path):
                        bin_files.append(path)
        if bin_files:
            return max(bin_files, key=os.path.getmtime)

    return ''


def normalize_path(path):
    return os.path.abspath(os.path.expanduser(path))


def resolve_map_file(cli_map_file, config_file, package_dir, map_dir):
    if cli_map_file:
        map_file = normalize_path(cli_map_file)
        if not os.path.isfile(map_file):
            return 'cli', map_file, f'CLI map_file does not exist: {map_file}'
        return 'cli', map_file, ''

    yaml_map_file, config_error = yaml_relocalization_map(config_file)
    if config_error:
        return 'yaml', '', config_error
    if yaml_map_file:
        map_file = normalize_path(yaml_map_file)
        if not os.path.isfile(map_file):
            return 'yaml', map_file, (
                f'YAML relocalization_map_abs_path does not exist: {map_file}'
            )
        return 'yaml', map_file, ''

    configured_source, configured_dir = configured_map_dir(map_dir)
    if configured_source and not os.path.isdir(configured_dir):
        return 'map_dir', configured_dir, (
            f'{configured_source} does not exist or is not a directory: {configured_dir}'
        )

    map_file = latest_bin_map(package_dir, map_dir)
    if not map_file:
        searched = map_search_directories(package_dir, map_dir)
        search_description = ', '.join(searched) if searched else '<no existing directories>'
        return (
            'auto latest',
            '',
            f'No .bin relocalization map found in: {search_description}',
        )

    return 'auto latest', map_file, ''


def abort_actions(reason):
    return [
        LogInfo(msg=f'[ERROR] {reason}'),
        EmitEvent(event=Shutdown(reason=reason)),
    ]


def launch_setup(context, *args, **kwargs):
    package_dir = get_package_share_directory('odin_ros_driver')
    config_file = LaunchConfiguration('config_file').perform(context).strip()
    map_dir = LaunchConfiguration('map_dir').perform(context).strip()
    cli_map_file = LaunchConfiguration('map_file').perform(context).strip()
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context)

    if not os.path.isfile(config_file):
        return abort_actions(f'Relocalization config file does not exist: {config_file}')

    source, map_file, error = resolve_map_file(
        cli_map_file, config_file, package_dir, map_dir
    )
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
    package_dir = get_package_share_directory('odin_ros_driver')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_dir, 'config', 'control_relocalization.yaml'),
        description='Path to the relocalization control config YAML file.',
    )

    map_dir_arg = DeclareLaunchArgument(
        'map_dir',
        default_value='',
        description=(
            'Optional directory to scan for the newest .bin map when map_file '
            'and relocalization_map_abs_path are empty. ODIN_MAP_DIR is used next.'
        ),
    )

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description=(
            'Optional relocalization map .bin path. '
            'Empty uses YAML path, then newest map in map_dir/ODIN_MAP_DIR.'
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
        config_file_arg,
        map_dir_arg,
        map_file_arg,
        rviz_config_arg,
        launch_rviz_arg,
        OpaqueFunction(function=launch_setup),
    ])
