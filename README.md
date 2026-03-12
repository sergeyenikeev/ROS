# PatrolBot

PatrolBot - практический ROS 2 проект мобильного робота для патрулирования помещений на базе Raspberry Pi, 2D LiDAR и дифференциального привода.

Проект собран как реальный workspace, ориентированный на Ubuntu Server 24.04 LTS arm64 и ROS 2 Jazzy. Целевая директория развёртывания и всех эксплуатационных скриптов зафиксирована как `/home/ubuntu/ROS`.

## Что умеет текущая версия

- принимать команды движения через `/cmd_vel`;
- публиковать одометрию, `joint_states` и диагностику;
- запускаться в `mock` и `hardware` режимах;
- публиковать тестовый `LaserScan` без реального LiDAR;
- строить карту через `slam_toolbox`;
- сохранять и загружать карту;
- локализоваться через `AMCL`;
- запускать `Nav2`;
- ехать к точкам по одной через `NavigateToPose`;
- выполнять патруль по YAML-маршруту с таймаутами и повторами;
- публиковать статус миссии и принимать команды `/patrol/start` и `/patrol/stop`;
- запускать unit и integration тесты.

## Состав пакетов

- `patrolbot_description` - URDF/Xacro, TF и запуск `robot_state_publisher`.
- `patrolbot_base` - базовая платформа, одометрия, драйверный слой, диагностика.
- `patrolbot_bringup` - launch-профили системы.
- `patrolbot_navigation` - конфиги `AMCL`, `Nav2` и тестовая карта.
- `patrolbot_slam` - конфиг `slam_toolbox`.
- `patrolbot_mission_manager` - state machine патрулирования и action-клиент `Nav2`.
- `patrolbot_interfaces` - пользовательские `msg/srv`.
- `patrolbot_utils` - логирование, валидаторы, загрузчик маршрутов, mock-сканер.
- `patrolbot_system_tests` - integration и launch-тесты, mock Nav2 action server.

## Требования

- Raspberry Pi 4/5 или совместимая ARM64 платформа.
- Ubuntu Server 24.04 LTS arm64.
- ROS 2 Jazzy Jalisco.
- `colcon`, `ament_cmake`, `rosdep`.
- 2D LiDAR с ROS 2 драйвером, публикующим `/scan`.
- Дифференциальное шасси с энкодерами или эквивалентной обратной связью по колёсам.

## Быстрый старт

1. Склонировать репозиторий в `/home/ubuntu/ROS`.
2. Установить зависимости:

```bash
/home/ubuntu/ROS/scripts/install_dependencies.sh
```

3. Собрать workspace:

```bash
/home/ubuntu/ROS/scripts/build_workspace.sh
```

4. Подключить окружение:

```bash
source /home/ubuntu/ROS/scripts/source_workspace.sh
```

## Сборка

Ручная команда сборки:

```bash
cd /home/ubuntu/ROS
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --event-handlers console_direct+
```

Готовый скрипт:

```bash
/home/ubuntu/ROS/scripts/build_workspace.sh
```

## Профили запуска

### Mock bringup

```bash
cd /home/ubuntu/ROS
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ROS/install/setup.bash
ros2 launch patrolbot_bringup mock_bringup.launch.py
```

### Hardware bringup

```bash
cd /home/ubuntu/ROS
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ROS/install/setup.bash
ros2 launch patrolbot_bringup hardware_bringup.launch.py
```

### Картографирование

```bash
/home/ubuntu/ROS/scripts/run_slam.sh false info
```

### Сохранение карты

```bash
/home/ubuntu/ROS/scripts/save_map.sh patrolbot_map
```

Карта будет сохранена в:

- `/home/ubuntu/ROS/maps/patrolbot_map.yaml`
- `/home/ubuntu/ROS/maps/patrolbot_map.pgm`

### Локализация

```bash
/home/ubuntu/ROS/scripts/run_localization.sh /home/ubuntu/ROS/maps/patrolbot_map.yaml false info
```

### Навигация

```bash
/home/ubuntu/ROS/scripts/run_navigation.sh /home/ubuntu/ROS/maps/patrolbot_map.yaml false info
```

### Патрулирование

```bash
/home/ubuntu/ROS/scripts/run_patrol.sh \
  /home/ubuntu/ROS/maps/patrolbot_map.yaml \
  /home/ubuntu/ROS/src/patrolbot_mission_manager/config/patrol_route.yaml \
  false \
  info
```

После запуска миссию можно включить сервисом:

```bash
source /home/ubuntu/ROS/scripts/source_workspace.sh
ros2 service call /patrol/start patrolbot_interfaces/srv/StartPatrol "{restart_if_running: false}"
```

Остановить миссию:

```bash
source /home/ubuntu/ROS/scripts/source_workspace.sh
ros2 service call /patrol/stop patrolbot_interfaces/srv/StopPatrol "{}"
```

## Как строить карту

1. Запустить `slam` профиль.
2. Дать роботу проехать по помещению.
3. Убедиться, что `/scan`, `/odom` и TF стабильны.
4. Сохранить карту через `/home/ubuntu/ROS/scripts/save_map.sh`.
5. Проверить, что появились файлы карты в `/home/ubuntu/ROS/maps`.

## Как запускать локализацию и навигацию

1. Подготовить карту `*.yaml`.
2. Запустить локализацию через `/home/ubuntu/ROS/scripts/run_localization.sh`.
3. Убедиться, что `AMCL` публикует `map -> odom`.
4. Запустить навигацию через `/home/ubuntu/ROS/scripts/run_navigation.sh` или полный patrol-профиль.

## Тестирование

Полный набор тестов:

```bash
/home/ubuntu/ROS/scripts/run_tests.sh
```

Только unit-тесты:

```bash
/home/ubuntu/ROS/scripts/run_unit_tests.sh
```

Отчёт:

```bash
/home/ubuntu/ROS/scripts/generate_test_report.sh
```

## Логи и отладка

Для запуска с подробными логами используйте аргумент `log_level:=debug`:

```bash
source /home/ubuntu/ROS/scripts/source_workspace.sh
ros2 launch patrolbot_bringup patrol.launch.py \
  map:=/home/ubuntu/ROS/maps/patrolbot_map.yaml \
  mission_file:=/home/ubuntu/ROS/src/patrolbot_mission_manager/config/patrol_route.yaml \
  use_mock_hardware:=false \
  log_level:=debug
```

Ключевые сообщения:

- `[узел]` - жизненный цикл нод;
- `[параметры]` - загрузка конфигурации;
- `[база]` - движение и watchdog;
- `[nav2]` - цели, feedback и результаты;
- `[миссия]` - переходы state machine;
- `[ошибка]` - отказ или некорректная конфигурация.

Подробности описаны в [LOGGING](docs/LOGGING.md).

## Ограничения

- реальный `serial`-драйвер базы пока заготовка и требует адаптации под конкретный контроллер;
- в текущей среде не было доступно ROS 2 окружение, поэтому сборка и запуск здесь не подтверждались;
- параметры `AMCL` и `Nav2` являются стартовой конфигурацией и требуют калибровки на железе;
- реальный драйвер LiDAR в проект не включён, ожидается внешний ROS 2 источник `/scan`.

## Типовые проблемы и решения

### Нет `/scan`

- проверить запуск ROS 2 драйвера LiDAR;
- для изоляционной проверки использовать `mock_bringup.launch.py`;
- убедиться, что `laser_frame` совпадает с конфигом драйвера.

### Нет TF `map -> odom`

- проверить, что запущен `AMCL` или `slam_toolbox`;
- убедиться, что карта корректно загружена;
- проверить, что `/odom` публикуется базовой платформой.

### Миссия зависает в `WAITING_FOR_NAV2`

- проверить, что навигационный стек действительно запущен;
- убедиться, что доступен action `/navigate_to_pose`;
- посмотреть параметр `nav2_wait_timeout_sec`.

### Робот уходит в таймаут waypoint

- проверить локализацию и наличие плана в `Nav2`;
- увеличить `timeout_sec` в YAML маршрута;
- проверить ширину робота и параметры инфляции costmap.

## Документация

- [Архитектура](docs/ARCHITECTURE.md)
- [Технический стек](docs/TECH_STACK.md)
- [Настройка Raspberry Pi](docs/SETUP_RASPBERRY_PI.md)
- [Runbook оператора](docs/RUNBOOK.md)
- [Тестирование](docs/TESTING.md)
- [Логирование](docs/LOGGING.md)
- [Список функций](docs/FEATURES.md)
- [Список багов](docs/BUGS.md)
