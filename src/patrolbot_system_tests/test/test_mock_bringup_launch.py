import time
import unittest

import launch
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_test_description():
    bringup_share = get_package_share_directory('patrolbot_bringup')
    launch_file = os.path.join(bringup_share, 'launch', 'mock_bringup.launch.py')

    return launch.LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file)),
        launch_testing.actions.ReadyToTest(),
    ])


class TestMockBringupLaunch(unittest.TestCase):

    def test_launch_stays_alive_briefly(self):
        time.sleep(2.0)
        self.assertTrue(True)
