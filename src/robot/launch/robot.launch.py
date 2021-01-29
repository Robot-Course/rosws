from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    ROBOT_MODEL = os.getenv('ROBOT_MODEL', 'lunar.1')

    descriptions = [
        DeclareLaunchArgument('imu', default_value='false'),
        Node(
            package='imu',
            executable='node',
            parameters=[LaunchConfiguration('imu_param_file',
                                            default=os.path.join(get_package_share_directory('imu'), 'param',
                                                                 ROBOT_MODEL + '.yaml'))],
            condition=IfCondition(LaunchConfiguration('imu'))
        ),

        DeclareLaunchArgument('motor', default_value='false'),
        Node(
            package='motor',
            executable='node',
            parameters=[LaunchConfiguration('motor_param_file',
                                            default=os.path.join(get_package_share_directory('motor'), 'param',
                                                                 ROBOT_MODEL + '.yaml'))],
            condition=IfCondition(LaunchConfiguration('motor'))
        ),

        DeclareLaunchArgument('lidar', default_value='false'),
        Node(
            package='lidar',
            executable='delta_lidar_node',
            parameters=[LaunchConfiguration('lidar_param_file',
                                            default=os.path.join(get_package_share_directory('lidar'),'param',
                                                                 ROBOT_MODEL + '.yaml'))],
            condition=IfCondition(LaunchConfiguration('lidar'))
        ),

        DeclareLaunchArgument('joystick', default_value='false'),
        Node(
            package='joystick_controller',
            executable='node',
            parameters=[LaunchConfiguration('joystick_param_file',
                                            default=os.path.join(get_package_share_directory('joystick_controller'),
                                                                 'param',
                                                                 ROBOT_MODEL + '.yaml'))],
            condition=IfCondition(LaunchConfiguration('joystick'))
        ),

        DeclareLaunchArgument('camera', default_value='false'),
        Node(
            package='camera',
            executable='node',
            condition=IfCondition(LaunchConfiguration('camera'))
        ),

        DeclareLaunchArgument('localization', default_value='false'),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[os.path.join(get_package_share_directory('robot'), 'param', 'robot_localization.yaml')],
            condition=IfCondition(LaunchConfiguration('localization'))
        )
    ]

    bridge_launch_dir = LaunchConfiguration(
        'bridge_launch_dir',
        default=os.path.join(get_package_share_directory('rosbridge_server'), 'launch'))

    descriptions.append(IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([bridge_launch_dir, '/rosbridge_websocket_launch.xml']),
        launch_arguments=[
            ('max_message_size', '10000000')
        ]
    ))

    return LaunchDescription(descriptions)
