# USAGE: ros2 launch odin_ros_driver odin1_ros2.launch.py
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(config_file):
    with open(config_file, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def get_custom_map_mode(config_file):
    params = load_yaml(config_file)
    return params.get('register_keys', {}).get('custom_map_mode', 0)


def load_config_params(config_file, calib_file_path, map_file):
    params = load_yaml(config_file)
    params.pop('pcd_map', None)
    if map_file:
        params.setdefault('register_keys', {})['relocalization_map_abs_path'] = map_file
    params['calib_file_path'] = calib_file_path
    params['config_file'] = config_file
    params['map_file'] = map_file
    return params


def cli_value(context, name):
    return LaunchConfiguration(name).perform(context)


def clean_library_path():
    system_libusb_paths = ['/usr/lib/x86_64-linux-gnu', '/lib/x86_64-linux-gnu']
    ros_library_paths = []
    for env_name in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH'):
        for prefix in os.environ.get(env_name, '').split(os.pathsep):
            if not prefix:
                continue
            lib_path = os.path.join(prefix, 'lib')
            if os.path.isdir(lib_path):
                ros_library_paths.append(lib_path)

    existing_paths = os.environ.get('LD_LIBRARY_PATH', '').split(os.pathsep)
    ordered_paths = system_libusb_paths + ros_library_paths + existing_paths
    clean_paths = []

    for path in ordered_paths:
        if not path:
            continue
        if path not in clean_paths:
            clean_paths.append(path)

    return os.pathsep.join(clean_paths)


def select_rviz_config(config_file, rviz_config, package_dir):
    if rviz_config != 'auto':
        return rviz_config

    mode = get_custom_map_mode(config_file)
    mode_rviz_files = {
        0: 'odin_odom.rviz',
        1: 'odin_mapping.rviz',
        2: 'odin_relocalization.rviz',
    }
    return os.path.join(package_dir, 'config', mode_rviz_files.get(mode, 'odin_odom.rviz'))


def launch_setup(context, *args, **kwargs):
    package_dir = get_package_share_directory('odin_ros_driver')
    config_file = cli_value(context, 'config_file')
    map_file = cli_value(context, 'map_file')
    rviz_config = select_rviz_config(
        config_file,
        cli_value(context, 'rviz_config'),
        package_dir,
    )
    calib_file_path = os.path.join(package_dir, 'config', 'calib.yaml')

    host_sdk_library_path = clean_library_path()

    host_sdk_node = Node(
        package='odin_ros_driver',
        executable='host_sdk_sample',
        name='host_sdk_sample',
        output='screen',
        additional_env={'LD_LIBRARY_PATH': host_sdk_library_path},
        sigterm_timeout='20.0',
        sigkill_timeout='5.0',
        parameters=[{
            'config_file': config_file,
            'map_file': map_file,
        }]
    )

    pcd2depth_params = load_config_params(config_file, calib_file_path, map_file)
    pcd2depth_node = Node(
        package='odin_ros_driver',
        executable='pcd2depth_ros2_node',
        name='pcd2depth_ros2_node',
        output='screen',
        parameters=[pcd2depth_params]
    )

    reprojection_params = load_config_params(config_file, calib_file_path, map_file)
    cloud_reprojection_node = Node(
        package='odin_ros_driver',
        executable='cloud_reprojection_ros2_node',
        name='cloud_reprojection_ros2_node',
        output='screen',
        parameters=[reprojection_params]
    )

    overlay_params = load_config_params(config_file, calib_file_path, map_file)
    image_overlay_node = Node(
        package='odin_ros_driver',
        executable='image_overlay_node',
        name='image_overlay_node',
        output='screen',
        parameters=[overlay_params]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    actions = [
        host_sdk_node,
        pcd2depth_node,
        cloud_reprojection_node,
        image_overlay_node,
    ]
    actions.append(rviz_node)
    return actions


def generate_launch_description():
    package_dir = get_package_share_directory('odin_ros_driver')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_dir, 'config', 'control_odom.yaml'),
        description='Path to the control config YAML file'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='auto',
        description='Path to RViz2 config file, or auto to match custom_map_mode'
    )

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Override relocalization map .bin file path; empty uses YAML value'
    )

    return LaunchDescription([
        config_file_arg,
        rviz_config_arg,
        map_file_arg,
        OpaqueFunction(function=launch_setup),
    ])
