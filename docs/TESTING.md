# Тестирование PatrolBot

## 1. Что покрыто

### Unit-тесты

- `patrolbot_utils/test/test_parameter_validation.cpp`
  - проверка валидаторов параметров;
  - форматирование ошибок.

- `patrolbot_utils/test/test_route_loader.cpp`
  - загрузка валидного YAML маршрута;
  - обработка некорректных значений.

- `patrolbot_base/test/test_odometry_integrator.cpp`
  - движение по прямой;
  - разворот на месте.

- `patrolbot_mission_manager/test/test_mission_state_machine.cpp`
  - ожидание Nav2;
  - повтор точки;
  - завершение с ошибкой;
  - переход к следующей точке при `continue_on_error=true`.

### Integration и launch-тесты

- `mock_bringup` smoke-test;
- `slam` smoke-test;
- `localization` smoke-test;
- `navigation` smoke-test;
- patrol success сценарий через mock Nav2;
- patrol retry сценарий через mock Nav2;
- patrol timeout сценарий через mock Nav2.

## 2. Что не покрыто

- реальный обмен по serial-порту с моторным контроллером;
- физический LiDAR;
- реальные задержки Raspberry Pi под нагрузкой;
- точность локализации на конкретном помещении;
- качество картографирования на реальном маршруте.

## 3. Какие моки используются

### `mock_scan_publisher`

Используется для:

- запуска `mock_bringup`;
- smoke-тестов launch;
- базовой проверки конвейера `scan -> slam/nav2`.

### `mock_nav2_action_server`

Используется для:

- проверки mission manager без настоящего Nav2;
- сценариев:
  - success;
  - abort + retry;
  - timeout.

## 4. Команды запуска тестов

Полный набор:

```bash
/home/ubuntu/ROS/scripts/run_tests.sh
```

Только unit:

```bash
/home/ubuntu/ROS/scripts/run_unit_tests.sh
```

Отчёт:

```bash
/home/ubuntu/ROS/scripts/generate_test_report.sh
```

Ручной запуск:

```bash
cd /home/ubuntu/ROS
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ROS/install/setup.bash
colcon test --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
```

## 5. Ограничения тестовой среды

В текущей рабочей среде не были доступны:

- `ros2`;
- `colcon`;
- компилятор C++;
- реальное Raspberry Pi железо.

Поэтому здесь проверены только:

- структура файлов;
- синтаксис Python launch-тестов через `py_compile`;
- связность проектной структуры и git-истории.

Полноценный прогон тестов нужно выполнить вручную на Ubuntu 24.04 с ROS 2 Jazzy.
