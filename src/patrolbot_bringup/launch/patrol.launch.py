from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Полный профиль PatrolBot: локализация, Nav2 и прикладная логика маршрута.
    # Такой запуск удобен для оператора, которому нужен один вход в систему,
    # а не ручная сборка нескольких отдельных launch-файлов.
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    map_yaml = LaunchConfiguration('map')
    mission_file = LaunchConfiguration('mission_file')
    log_level = LaunchConfiguration('log_level')

    bringup_share = get_package_share_directory('patrolbot_bringup')
    navigation_share = get_package_share_directory('patrolbot_navigation')
    mission_share = get_package_share_directory('patrolbot_mission_manager')

    navigation_launch = os.path.join(bringup_share, 'launch', 'navigation.launch.py')
    default_mission = os.path.join(mission_share, 'config', 'patrol_route.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_mock_hardware', default_value='true'),
        # По умолчанию используется тестовая карта, но в реальной эксплуатации
        # сюда обычно передаётся путь к сохранённой карте объекта.
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(navigation_share, 'maps', 'test_map.yaml'),
        ),
        # YAML-миссия пробрасывается как параметр, чтобы оператор мог быстро
        # переключать сценарии патрулирования без правки launch-файла.
        DeclareLaunchArgument('mission_file', default_value=default_mission),
        DeclareLaunchArgument('log_level', default_value='info'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            # Сначала поднимается полный стек навигации, а уже затем mission manager,
            # иначе он может стартовать раньше, чем появится /navigate_to_pose.
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_mock_hardware': use_mock_hardware,
                'map': map_yaml,
                'log_level': log_level,
            }.items(),
        ),
        Node(
            # Mission manager получает только минимум параметров через launch:
            # путь к маршруту и режим времени. Всё остальное берётся из своих
            # внутренних параметров и конфигурации.
            package='patrolbot_mission_manager',
            executable='patrolbot_mission_manager_node',
            name='patrolbot_mission_manager',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'mission_file': mission_file,
                }
            ],
        ),
    ])
