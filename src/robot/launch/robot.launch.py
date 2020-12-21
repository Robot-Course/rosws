from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
import argparse


def generate_launch_description():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-model', type=str, default='lunar.1')
    parser.add_argument('--all', action='store_true', default=False)
    parser.add_argument('--imu', action='store_true', default=False)
    parser.add_argument('--motor', action='store_true', default=False)
    parser.add_argument('--lidar', action='store_true', default=False)
    parser.add_argument('--joystick', action='store_true', default=False)
    parser.add_argument('--camera', action='store_true', default=False)
    args, _ = parser.parse_known_args()

    descriptions = []

    if args.imu or args.all:
        imu_param_file = LaunchConfiguration(
            'imu_param_file',
            default=os.path.join(
                get_package_share_directory('imu'),
                'param',
                args.robot_model + '.yaml'))
        descriptions.append(Node(
            package='imu',
            node_executable='node',
            parameters=[imu_param_file]))

    if args.motor or args.all:
        motor_param_file = LaunchConfiguration(
            'motor_param_file',
            default=os.path.join(
                get_package_share_directory('motor'),
                'param',
                args.robot_model + '.yaml'))
        descriptions.append(Node(
            package='motor',
            node_executable='node',
            parameters=[motor_param_file]))

    if args.lidar or args.all:
        lidar_param_file = LaunchConfiguration(
            'lidar_param_file',
            default=os.path.join(
                get_package_share_directory('lidar'),
                'param',
                args.robot_model + '.yaml'))
        descriptions.append(Node(
            package='lidar',
            node_executable='delta_lidar_node',
            parameters=[lidar_param_file]))

    if args.joystick or args.all:
        joystick_param_file = LaunchConfiguration(
            'joystick_param_file',
            default=os.path.join(
                get_package_share_directory('joystick_controller'),
                'param',
                args.robot_model + '.yaml'))
        descriptions.append(Node(
            package='joystick_controller',
            node_executable='node',
            parameters=[joystick_param_file]))

    if args.camera or args.all:
        descriptions.append(Node(
            package='camera',
            node_executable='node'))

    bridge_launch_dir = LaunchConfiguration(
        'bridge_launch_dir',
        default=os.path.join(get_package_share_directory('rosbridge_server'), 'launch'))

    descriptions.append(IncludeLaunchDescription(
            FrontendLaunchDescriptionSource([bridge_launch_dir, '/rosbridge_websocket_launch.xml'])
        ))

    return LaunchDescription(descriptions)
