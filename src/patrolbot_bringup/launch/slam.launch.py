from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch-профиль SLAM выбирает только источник базовой платформы:
    # mock-окружение или реальное железо. Сама логика картографирования поверх
    # этого источника остаётся одинаковой.
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    log_level = LaunchConfiguration('log_level')

    bringup_share = get_package_share_directory('patrolbot_bringup')
    slam_share = get_package_share_directory('patrolbot_slam')

    mock_launch = os.path.join(bringup_share, 'launch', 'mock_bringup.launch.py')
    hardware_launch = os.path.join(bringup_share, 'launch', 'hardware_bringup.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_mock_hardware', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mock_launch),
            # Mock-режим удобен для smoke-проверки launch и для ранней настройки
            # SLAM-пайплайна без реального робота.
            condition=IfCondition(use_mock_hardware),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'log_level': log_level,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_launch),
            # В аппаратном режиме берутся реальные источники одометрии и датчиков.
            condition=UnlessCondition(use_mock_hardware),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'log_level': log_level,
            }.items(),
        ),
        Node(
            # Используется асинхронный режим slam_toolbox как практичный вариант
            # для живого картографирования в помещении на PatrolBot.
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                os.path.join(slam_share, 'config', 'slam_toolbox_async.yaml'),
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
