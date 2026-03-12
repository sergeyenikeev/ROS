from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('patrolbot_description')
    default_xacro = os.path.join(package_share, 'urdf', 'patrolbot.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_frequency = LaunchConfiguration('publish_frequency')
    xacro_file = LaunchConfiguration('xacro_file')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Использовать ли simulated time.',
        ),
        DeclareLaunchArgument(
            'publish_frequency',
            default_value='30.0',
            description='Частота публикации состояния робота.',
        ),
        DeclareLaunchArgument(
            'xacro_file',
            default_value=default_xacro,
            description='Абсолютный путь до файла Xacro.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'publish_frequency': publish_frequency,
                    'robot_description': robot_description,
                }
            ],
        ),
    ])
