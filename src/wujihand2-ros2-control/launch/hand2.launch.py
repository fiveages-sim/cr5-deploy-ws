"""Wuji Hand2 + BasicJointController launch (mock / real).

Wuji-specific xacro / HI parameters live here — not in shared basic_joint_controller/hand.launch.py.

Real hardware connection (launch args, priority):
  1) device_address:=IP:port
  2) serial_number:=SN  (if device_address empty)
  3) both empty → SDK wuji_scan + match direction (1=left, -1=right)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
import xacro

from robot_common_launch import load_robot_config, create_rmw_zenohd_node

HAND_NAME = 'wuji'
HAND_TYPE = 'hand2'


def launch_setup(context, *args, **kwargs):
    direction = context.launch_configurations.get('direction', '1')
    hardware = context.launch_configurations.get('hardware', 'mock_components')
    use_sim_time = hardware in ['gz', 'isaac']

    hand_side = 'left' if direction == '1' else 'right'
    print(f'[INFO] Wuji Hand2: {hand_side} (direction={direction}, hardware={hardware})')

    hand_pkg_path = get_package_share_directory(f'{HAND_NAME}_description')
    hand_xacro_path = os.path.join(hand_pkg_path, 'xacro', 'ros2_control', 'hand.xacro')
    if not os.path.exists(hand_xacro_path):
        print(f'[ERROR] Hand xacro not found: {hand_xacro_path}')
        return []

    mappings = {
        'type': HAND_TYPE,
        'direction': direction,
        'ros2_control_hardware_type': hardware,
        'device_address': context.launch_configurations.get('device_address', ''),
        'serial_number': context.launch_configurations.get('serial_number', ''),
        'mit_kp': context.launch_configurations.get('mit_kp', '3.0'),
        'mit_kd': context.launch_configurations.get('mit_kd', '0.05'),
        'effort_limit': context.launch_configurations.get('effort_limit', '1.5'),
        'read_feedback': context.launch_configurations.get('read_feedback', 'true'),
        'require_initial_feedback': context.launch_configurations.get(
            'require_initial_feedback', 'true'),
        'command_deadband': context.launch_configurations.get('command_deadband', '0.0'),
        'connect_timeout_ms': context.launch_configurations.get('connect_timeout_ms', '5000'),
        'enable_timeout_s': context.launch_configurations.get('enable_timeout_s', '5.0'),
    }
    if hardware == 'gz':
        mappings['gazebo'] = 'true'

    try:
        robot_description = xacro.process_file(hand_xacro_path, mappings=mappings).toxml()
    except Exception as exc:
        print(f'[ERROR] Failed to process hand xacro: {exc}')
        return []

    _, ros2_controllers_path, _meta = load_robot_config(HAND_NAME, 'ros2_control', HAND_TYPE)
    if ros2_controllers_path is None:
        print(f'[ERROR] Controllers config not found for {HAND_NAME}/{HAND_TYPE}')
        return []

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'publish_frequency': 100.0,
            'use_tf_static': True,
            'robot_description': robot_description,
        }],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            ros2_controllers_path,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('/controller_manager/robot_description', '/robot_description')],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    hand_joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hand_joint_controller'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    nodes = [
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        hand_joint_controller_spawner,
    ]

    use_rviz = context.launch_configurations.get('use_rviz', 'true').lower() == 'true'
    if use_rviz:
        rviz_config = os.path.join(
            get_package_share_directory('basic_joint_controller'), 'config', 'hand.rviz')
        if os.path.exists(rviz_config):
            nodes.append(Node(
                package='rviz2',
                executable='rviz2',
                output='log',
                arguments=['-d', rviz_config],
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'joint_controllers': ['hand_joint_controller']},
                ],
            ))
        else:
            print(f'[WARN] RViz config not found: {rviz_config}')

    rmw_zenohd_node = create_rmw_zenohd_node()
    if rmw_zenohd_node is not None:
        nodes.insert(0, rmw_zenohd_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'direction', default_value='1',
            description='1=left, -1=right'),
        DeclareLaunchArgument(
            'hardware', default_value='mock_components',
            description='mock_components | real | gz | isaac'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz'),
        DeclareLaunchArgument(
            'device_address', default_value='',
            description='Hand2 IP:port (e.g. 192.168.1.110:50001)'),
        DeclareLaunchArgument(
            'serial_number', default_value='',
            description='Hand2 serial (if device_address empty)'),
        DeclareLaunchArgument('mit_kp', default_value='3.0'),
        DeclareLaunchArgument('mit_kd', default_value='0.05'),
        DeclareLaunchArgument('effort_limit', default_value='1.5'),
        DeclareLaunchArgument('read_feedback', default_value='true'),
        DeclareLaunchArgument('require_initial_feedback', default_value='true'),
        DeclareLaunchArgument('command_deadband', default_value='0.0'),
        DeclareLaunchArgument('connect_timeout_ms', default_value='5000'),
        DeclareLaunchArgument('enable_timeout_s', default_value='5.0'),
        OpaqueFunction(function=launch_setup),
    ])
