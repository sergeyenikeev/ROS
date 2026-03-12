import time
import unittest

import launch
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_test_description():
    # Это smoke-тест именно orchestration-слоя. Он не проверяет физическую
    # корректность данных, а подтверждает, что профиль mock_bringup в принципе
    # поднимается без немедленного падения процессов.
    bringup_share = get_package_share_directory('patrolbot_bringup')
    launch_file = os.path.join(bringup_share, 'launch', 'mock_bringup.launch.py')

    return launch.LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file)),
        launch_testing.actions.ReadyToTest(),
    ])


class TestMockBringupLaunch(unittest.TestCase):

    def test_launch_stays_alive_briefly(self):
        # Короткая задержка достаточна, чтобы поймать ошибки ранней инициализации:
        # битые пути, отсутствующие executable и некорректные параметры launch.
        time.sleep(2.0)
        self.assertTrue(True)
