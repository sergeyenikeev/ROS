# Архитектура PatrolBot

## 1. Общая схема

PatrolBot построен как набор ROS 2 пакетов с явным разделением ответственности:

- `patrolbot_description` задаёт геометрию робота и статические фреймы.
- `patrolbot_base` отвечает за движение, одометрию, `joint_states` и диагностику.
- `patrolbot_utils` предоставляет общие утилиты, загрузчик маршрутов и mock-источники данных.
- `patrolbot_slam` инкапсулирует конфигурацию `slam_toolbox`.
- `patrolbot_navigation` инкапсулирует карту, `AMCL` и `Nav2`.
- `patrolbot_mission_manager` реализует прикладную state machine патруля.
- `patrolbot_bringup` собирает подсистемы в профили запуска.
- `patrolbot_system_tests` содержит тестовую оркестрацию и mock Nav2 action server.

Такое разбиение выбрано, чтобы:

- отделить железо от прикладной логики;
- упростить тестирование без Raspberry Pi;
- минимизировать связность между SLAM, локализацией и патрулём;
- позволить запускать subsystem-by-subsystem, а не только весь стек целиком.

## 2. Ноды

### `patrolbot_base`

Пакет запускает узел `patrolbot_base`:

- подписывается на `/cmd_vel`;
- ограничивает линейную и угловую скорости;
- включает watchdog потери команд;
- переводит `Twist` в скорости колёс;
- работает через интерфейс `MotorDriver`;
- публикует `/odom`;
- публикует `/joint_states`;
- публикует `/diagnostics`;
- опционально публикует TF `odom -> base_footprint`.

### `robot_state_publisher`

Запускается из `patrolbot_description` и публикует:

- `base_footprint -> base_link`
- `base_link -> left_wheel_link`
- `base_link -> right_wheel_link`
- `base_link -> laser_frame`

### `mock_scan_publisher`

Запускается в `mock` профиле из `patrolbot_utils`:

- публикует синтетический `sensor_msgs/msg/LaserScan` в `/scan`;
- нужен для smoke-тестов и отладки без реального LiDAR.

### `slam_toolbox`

Запускается в профиле `slam`:

- подписывается на `/scan` и TF;
- строит карту;
- публикует `map -> odom`.

### `map_server` и `amcl`

Запускаются в профиле `localization`:

- `map_server` загружает сохранённую карту;
- `AMCL` локализует робота на карте;
- публикуется TF `map -> odom`.

### Узлы `Nav2`

Запускаются в профиле `navigation`:

- `planner_server`
- `controller_server`
- `behavior_server`
- `bt_navigator`
- `waypoint_follower`
- `velocity_smoother`

### `patrolbot_mission_manager`

Ключевая прикладная нода:

- загружает YAML-миссию;
- валидирует маршрут;
- ждёт доступность action `/navigate_to_pose`;
- отправляет waypoint-ы по одному;
- отслеживает feedback и результат;
- обрабатывает таймаут, повтор и остановку;
- публикует `/patrol/status`;
- обслуживает `/patrol/start` и `/patrol/stop`.

## 3. Топики

Основные топики проекта:

- `/cmd_vel` - входная команда движения.
- `/odom` - одометрия базовой платформы.
- `/joint_states` - состояние колёс.
- `/diagnostics` - диагностика узлов.
- `/scan` - данные 2D LiDAR.
- `/map` - карта помещения.
- `/tf` и `/tf_static` - дерево TF.
- `/patrol/status` - текущий статус миссии патрулирования.

## 4. Сервисы

- `/patrol/start` (`patrolbot_interfaces/srv/StartPatrol`)
- `/patrol/stop` (`patrolbot_interfaces/srv/StopPatrol`)

Назначение:

- `start` запускает миссию по YAML-маршруту;
- `stop` останавливает текущий патруль;
- `restart_if_running=true` инициирует отмену текущей цели и перезапуск миссии.

## 5. Actions

Используемый action:

- `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`)

Почему выбран именно он:

- позволяет контролировать каждую точку отдельно;
- удобно добавлять per-waypoint таймауты;
- проще реализовать повторы и детальные логи;
- проще тестировать через mock action server.

## 6. TF-дерево

Зафиксированная цепочка:

- `map -> odom -> base_footprint -> base_link -> laser_frame`

Распределение ответственности:

- `map -> odom` публикует `slam_toolbox` или `AMCL`;
- `odom -> base_footprint` публикует `patrolbot_base`;
- остальная часть цепочки задаётся URDF/Xacro и `robot_state_publisher`.

## 7. State machine миссии

Состояния:

- `IDLE`
- `VALIDATING_ROUTE`
- `WAITING_FOR_NAV2`
- `NAVIGATING`
- `RETRY_WAIT`
- `STOPPING`
- `COMPLETED`
- `FAILED`

Типовой сценарий:

1. оператор вызывает `/patrol/start`;
2. нода переходит в `VALIDATING_ROUTE`;
3. маршрут загружается и проверяется;
4. при отсутствии Nav2 нода переходит в `WAITING_FOR_NAV2`;
5. после появления action server начинается `NAVIGATING`;
6. при ошибке цели возможен `RETRY_WAIT`;
7. после последней точки миссия переходит в `COMPLETED`;
8. при неустранимой ошибке миссия переходит в `FAILED`.

## 8. Профили запуска

### `mock_bringup.launch.py`

- `robot_state_publisher`
- `patrolbot_base` в режиме `mock`
- `mock_scan_publisher`

### `hardware_bringup.launch.py`

- `robot_state_publisher`
- `patrolbot_base` в режиме `serial`
- внешний LiDAR ожидается отдельно

### `slam.launch.py`

- `mock_bringup` или `hardware_bringup`
- `slam_toolbox`

### `localization.launch.py`

- `mock_bringup` или `hardware_bringup`
- `map_server`
- `AMCL`
- `lifecycle_manager`

### `navigation.launch.py`

- `localization.launch.py`
- узлы `Nav2`

### `patrol.launch.py`

- `navigation.launch.py`
- `patrolbot_mission_manager`

## 9. Основные файлы конфигурации

- `src/patrolbot_base/config/base_mock.yaml`
- `src/patrolbot_base/config/base_hardware.yaml`
- `src/patrolbot_slam/config/slam_toolbox_async.yaml`
- `src/patrolbot_navigation/config/amcl.yaml`
- `src/patrolbot_navigation/config/nav2_params.yaml`
- `src/patrolbot_mission_manager/config/patrol_route.yaml`

## 10. Почему именно такой дизайн

### Собственный драйвер базы вместо `ros2_control`

- быстрее довести v1 до работающего прототипа;
- проще тестировать без контроллерного стека;
- проще отделить бизнес-логику odometry/diagnostics от конкретного оборудования.

### `AMCL` вместо режима локализации `slam_toolbox`

- более привычный и предсказуемый эксплуатационный режим;
- легче разделить картографирование и боевую навигацию;
- проще поддерживать и диагностировать.

### Выделенный `mission_manager`

- `Nav2` отвечает только за навигацию, а не за бизнес-логику обхода;
- таймауты, повторы и состояние миссии лежат в отдельном понятном модуле;
- прикладной код можно тестировать без настоящего робота.
