import importlib.util
import os
import tempfile
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('arm_control'))
    spec = importlib.util.spec_from_file_location('arm_model', share / 'launch/model.py')
    model = importlib.util.module_from_spec(spec); spec.loader.exec_module(model)
    config_path = Path(LaunchConfiguration('config').perform(context)).resolve()
    with config_path.open(encoding='utf-8') as stream:
        cfg = yaml.safe_load(stream)
    backend = LaunchConfiguration('backend').perform(context)
    if backend == 'from_config': backend = None
    if 'sdk_xml' in cfg['hardware']:
        xml = Path(cfg['hardware']['sdk_xml'])
        cfg['hardware']['sdk_xml'] = str(xml if xml.is_absolute() else config_path.parent / xml)
    urdf, controllers = model.build(cfg, backend)
    fd, params = tempfile.mkstemp(prefix='arm_controllers_', suffix='.yaml')
    with os.fdopen(fd, 'w') as stream:
        yaml.safe_dump(controllers, stream)
    def cleanup(_context):
        Path(params).unlink(missing_ok=True)
        return []
    actions = [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': urdf}]),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[{'robot_description': urdf}, params], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30']),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_trajectory_controller', '--inactive', '--controller-manager-timeout', '30'])]
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2',
                            arguments=['-d', str(share / 'config/arm.rviz')], output='screen'))
    if LaunchConfiguration('record').perform(context).lower() == 'true':
        actions.append(ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration('bag_path'),
                       '/joint_states', '/diagnostics', '/arm_hardware/timing',
                       '/joint_trajectory_controller/controller_state',
                       '/joint_trajectory_controller/joint_trajectory', '/robot_description'], output='screen'))
    return actions


def generate_launch_description():
    share = get_package_share_directory('arm_control')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'robot_robomaster.yaml')),
        DeclareLaunchArgument('backend', default_value='from_config', choices=['from_config', 'sim', 'igh', 'can_mock', 'socketcan', 'zcan', 'loong', 'rm_mock', 'rm_socketcan', 'rm_zcan']),
        DeclareLaunchArgument('record', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('bag_path', default_value='arm_bag'),
        OpaqueFunction(function=setup)])
