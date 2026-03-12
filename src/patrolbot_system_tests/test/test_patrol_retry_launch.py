import os
import time
import unittest

import launch
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from patrolbot_interfaces.msg import PatrolStatus
from patrolbot_interfaces.srv import StartPatrol


def generate_test_description():
    # Сценарий проверяет, что после первой ошибки mission manager переходит
    # в RETRY_WAIT, повторяет goal и затем успешно заканчивает миссию.
    system_tests_share = get_package_share_directory('patrolbot_system_tests')
    mission_file = os.path.join(system_tests_share, 'test', 'patrol_retry.yaml')

    return launch.LaunchDescription([
        Node(
            # Первая попытка завершается abort, вторая тем же mock-сервером
            # завершается успехом.
            package='patrolbot_system_tests',
            executable='mock_nav2_action_server',
            name='mock_nav2_action_server',
            output='screen',
            parameters=[{'result_sequence': 'abort,succeed'}],
        ),
        Node(
            package='patrolbot_mission_manager',
            executable='patrolbot_mission_manager_node',
            name='patrolbot_mission_manager',
            output='screen',
            parameters=[{
                'mission_file': mission_file,
                'status_publish_rate_hz': 10.0,
                'nav2_wait_timeout_sec': 5.0,
            }],
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestPatrolRetryLaunch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_patrol_retry_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_patrol_retries_and_completes(self):
        statuses = []
        self.subscription = self.node.create_subscription(PatrolStatus, '/patrol/status', statuses.append, 10)
        client = self.node.create_client(StartPatrol, '/patrol/start')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))

        request = StartPatrol.Request()
        request.restart_if_running = False
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertTrue(future.result().accepted)

        deadline = time.time() + 15.0
        # Фиксируем сам факт входа в RETRY_WAIT, иначе тест проверял бы только
        # финальный успех, но не саму механику повторной попытки.
        seen_retry_wait = False
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if statuses:
                seen_retry_wait = seen_retry_wait or any(item.state == 'RETRY_WAIT' for item in statuses)
                if statuses[-1].state == 'COMPLETED':
                    self.assertTrue(seen_retry_wait)
                    return

        self.fail('Миссия не прошла сценарий с повтором за ожидаемое время')
