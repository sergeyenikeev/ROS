from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Профиль локализации поднимает базовую платформу, загрузку карты и AMCL.
    # Он отделён от навигации, чтобы можно было отдельно проверять только
    # качество map -> odom и корректность начальной позы.
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    map_yaml = LaunchConfiguration('map')
    log_level = LaunchConfiguration('log_level')

    bringup_share = get_package_share_directory('patrolbot_bringup')
    navigation_share = get_package_share_directory('patrolbot_navigation')

    mock_launch = os.path.join(bringup_share, 'launch', 'mock_bringup.launch.py')
    hardware_launch = os.path.join(bringup_share, 'launch', 'hardware_bringup.launch.py')
    amcl_config = os.path.join(navigation_share, 'config', 'amcl.yaml')
    default_map = os.path.join(navigation_share, 'maps', 'test_map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_mock_hardware', default_value='true'),
        # По умолчанию используется тестовая карта пакета navigation, но в
        # реальной эксплуатации сюда передаётся сохранённая карта объекта.
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('log_level', default_value='info'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mock_launch),
            condition=IfCondition(use_mock_hardware),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'log_level': log_level,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_launch),
            condition=UnlessCondition(use_mock_hardware),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'log_level': log_level,
            }.items(),
        ),
        Node(
            # map_server отвечает только за загрузку готовой карты и никак не
            # занимается локализацией или навигацией сам по себе.
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}],
        ),
        Node(
            # AMCL выбран как основной локализатор на сохранённой карте для
            # практического indoor-сценария PatrolBot.
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[amcl_config, {'use_sim_time': use_sim_time}],
        ),
        Node(
            # Lifecycle manager переводит map_server и AMCL в активное состояние
            # без ручного управления их жизненным циклом.
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['map_server', 'amcl'],
                }
            ],
        ),
    ])
