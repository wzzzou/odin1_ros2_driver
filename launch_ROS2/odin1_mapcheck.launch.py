# USAGE: ros2 launch odin_ros_driver odin1_mapcheck.launch.py [map_file:=/path/to/map.bin] [pcd_map_file:=/path/to/map.pcd] [rviz_config:=auto]
import hashlib
import os
import tempfile

import yaml
from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
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


def resolve_rviz_config(package_share_dir, rviz_config, map_frame):
    if rviz_config != 'auto':
        if not os.path.isfile(rviz_config):
            return '', f'RViz config does not exist: {rviz_config}'
        return rviz_config, ''

    default_config = os.path.join(
        package_share_dir, 'config', 'odin_relocalization_mapcheck.rviz'
    )
    if map_frame == 'odin1_map':
        return default_config, ''

    try:
        with open(default_config, 'r', encoding='utf-8') as config_stream:
            content = config_stream.read()
        marker = 'Fixed Frame: odin1_map'
        if marker not in content:
            return '', f'RViz config has no expected fixed frame marker: {default_config}'
        content = content.replace(marker, f'Fixed Frame: {map_frame}', 1)
        cache_key = hashlib.sha256(map_frame.encode('utf-8')).hexdigest()[:12]
        generated_config = os.path.join(
            tempfile.gettempdir(),
            f'odin1_relocalization_mapcheck_{cache_key}.rviz',
        )
        with tempfile.NamedTemporaryFile(
            mode='w',
            encoding='utf-8',
            dir=tempfile.gettempdir(),
            prefix='odin1_relocalization_mapcheck_',
            suffix='.tmp',
            delete=False,
        ) as generated_stream:
            generated_stream.write(content)
            temporary_config = generated_stream.name
        os.replace(temporary_config, generated_config)
        return generated_config, ''
    except OSError as exc:
        return '', f'Failed to prepare RViz config for frame {map_frame}: {exc}'


def resolve_map_frame(config_file):
    try:
        with open(config_file, 'r', encoding='utf-8') as config_stream:
            params = yaml.safe_load(config_stream) or {}
    except (OSError, yaml.YAMLError) as exc:
        return '', f'Failed to read relocalization config {config_file}: {exc}'

    register_keys = params.get('register_keys', {})
    if not isinstance(register_keys, dict):
        return '', f'Invalid register_keys section in {config_file}'

    map_frame = register_keys.get('map_frame')
    if not isinstance(map_frame, str) or not map_frame.strip():
        return '', f'Invalid register_keys.map_frame in {config_file}'
    map_frame = map_frame.strip()
    if any(character.isspace() for character in map_frame):
        return '', f'register_keys.map_frame must not contain whitespace in {config_file}'
    return map_frame, ''


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
    config_file = LaunchConfiguration('config_file').perform(context).strip()
    map_dir = LaunchConfiguration('map_dir').perform(context).strip()
    map_file = LaunchConfiguration('map_file').perform(context).strip()
    cli_pcd_map_file = LaunchConfiguration('pcd_map_file').perform(context).strip()
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context)
    requested_rviz_config = LaunchConfiguration('rviz_config').perform(context).strip()
    if not os.path.isfile(config_file):
        return abort_actions(f'Relocalization config file does not exist: {config_file}')

    map_frame, error = resolve_map_frame(config_file)
    if error:
        return abort_actions(error)

    rviz_config, error = resolve_rviz_config(
        package_share_dir, requested_rviz_config, map_frame
    )
    if error:
        return abort_actions(error)

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
                'config_file': config_file,
                'map_dir': map_dir,
                'map_file': map_file,
                'rviz_config': rviz_config,
                'launch_rviz': launch_rviz,
            }.items(),
        ),
    ]

    if pcd_map_file:
        parent_death_guard = os.path.join(
            get_package_prefix('odin_ros_driver'),
            'lib',
            'odin_ros_driver',
            'odin_parent_death_guard',
        )
        actions.append(
            Node(
                package='odin_ros_driver',
                executable='static_pcd_map_publisher',
                name='static_pcd_map_publisher',
                exec_name='static_pcd_map_publisher',
                output='screen',
                prefix=parent_death_guard,
                parameters=[{
                    'pcd_map_file': pcd_map_file,
                    'topic_name': '/odin1/map_cloud',
                    'frame_id': map_frame,
                }],
            )
        )

    return actions


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
        description='Optional directory to scan for the newest .bin map.',
    )

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
        config_file_arg,
        map_dir_arg,
        map_file_arg,
        pcd_map_file_arg,
        rviz_config_arg,
        launch_rviz_arg,
        OpaqueFunction(function=launch_setup),
    ])
