from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    map_yaml = LaunchConfiguration('map')
    log_level = LaunchConfiguration('log_level')

    bringup_share = get_package_share_directory('patrolbot_bringup')
    navigation_share = get_package_share_directory('patrolbot_navigation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    localization_launch = os.path.join(bringup_share, 'launch', 'localization.launch.py')
    nav2_launch = os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_mock_hardware', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(navigation_share, 'maps', 'test_map.yaml'),
        ),
        DeclareLaunchArgument('log_level', default_value='info'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_mock_hardware': use_mock_hardware,
                'map': map_yaml,
                'log_level': log_level,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': os.path.join(navigation_share, 'config', 'nav2_params.yaml'),
            }.items(),
        ),
    ])
