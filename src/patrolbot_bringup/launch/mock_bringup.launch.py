from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    description_share = get_package_share_directory('patrolbot_description')
    base_share = get_package_share_directory('patrolbot_base')
    utils_share = get_package_share_directory('patrolbot_utils')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_share, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        Node(
            package='patrolbot_base',
            executable='patrolbot_base_node',
            name='patrolbot_base',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[os.path.join(base_share, 'config', 'base_mock.yaml')],
        ),
        Node(
            package='patrolbot_utils',
            executable='mock_scan_publisher',
            name='mock_scan_publisher',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[os.path.join(utils_share, 'config', 'mock_scan.yaml')],
        ),
    ])
