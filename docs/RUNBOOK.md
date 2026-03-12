# Runbook PatrolBot

## 1. Подготовить окружение

```bash
source /home/ubuntu/ROS/scripts/source_workspace.sh
```

Если workspace ещё не собран:

```bash
/home/ubuntu/ROS/scripts/build_workspace.sh
source /home/ubuntu/ROS/scripts/source_workspace.sh
```

## 2. Построить карту

Запуск:

```bash
/home/ubuntu/ROS/scripts/run_slam.sh false info
```

Дальше:

1. проехать роботом по помещению;
2. не допускать резких рывков и потери `/scan`;
3. контролировать, что карта стабильно растёт.

## 3. Сохранить карту

```bash
/home/ubuntu/ROS/scripts/save_map.sh patrolbot_map
```

Проверить:

```bash
ls -l /home/ubuntu/ROS/maps/patrolbot_map.*
```

## 4. Загрузить карту и запустить локализацию

```bash
/home/ubuntu/ROS/scripts/run_localization.sh /home/ubuntu/ROS/maps/patrolbot_map.yaml false info
```

Проверка:

```bash
source /home/ubuntu/ROS/scripts/source_workspace.sh
ros2 topic echo /amcl_pose --once
```

## 5. Запустить навигацию

```bash
/home/ubuntu/ROS/scripts/run_navigation.sh /home/ubuntu/ROS/maps/patrolbot_map.yaml false info
```

Проверка доступности action:

```bash
source /home/ubuntu/ROS/scripts/source_workspace.sh
ros2 action list | grep navigate_to_pose
```

## 6. Запустить патруль

```bash
/home/ubuntu/ROS/scripts/run_patrol.sh \
  /home/ubuntu/ROS/maps/patrolbot_map.yaml \
  /home/ubuntu/ROS/src/patrolbot_mission_manager/config/patrol_route.yaml \
  false \
  info
```

Старт миссии:

```bash
source /home/ubuntu/ROS/scripts/source_workspace.sh
ros2 service call /patrol/start patrolbot_interfaces/srv/StartPatrol "{restart_if_running: false}"
```

## 7. Остановить патруль

```bash
source /home/ubuntu/ROS/scripts/source_workspace.sh
ros2 service call /patrol/stop patrolbot_interfaces/srv/StopPatrol "{}"
```

## 8. Проверить текущее состояние

Статус миссии:

```bash
source /home/ubuntu/ROS/scripts/source_workspace.sh
ros2 topic echo /patrol/status
```

Диагностика:

```bash
source /home/ubuntu/ROS/scripts/source_workspace.sh
ros2 topic echo /diagnostics
```

TF:

```bash
source /home/ubuntu/ROS/scripts/source_workspace.sh
ros2 run tf2_ros tf2_echo odom base_footprint
```

## 9. Найти причину типовых ошибок

### Состояние `WAITING_FOR_NAV2`

Проверить:

```bash
source /home/ubuntu/ROS/scripts/source_workspace.sh
ros2 action list | grep navigate_to_pose
```

Если action нет:

- не запущен `navigation.launch.py`;
- `Nav2` упал на старте;
- некорректна карта или lifecycle узлов.

### Состояние `FAILED` после старта

Проверить:

- путь к файлу миссии;
- корректность YAML маршрута;
- наличие хотя бы одной точки;
- таймаут ожидания Nav2.

### Робот не двигается при наличии `/cmd_vel`

Проверить:

- параметр `driver_mode`;
- наличие `/odom`;
- сообщения `/diagnostics`;
- корректность `wheel_radius` и `wheel_base`.

### Невозможно сохранить карту

Проверить:

- что `slam_toolbox` действительно запущен;
- что каталог `/home/ubuntu/ROS/maps` доступен на запись;
- что карта уже накоплена, а `/scan` не пуст.

## 10. Рекомендуемый порядок эксплуатации

1. `hardware_bringup` и проверка датчиков.
2. `slam` и сохранение карты.
3. `localization` на сохранённой карте.
4. `navigation`.
5. `patrol`.

Такой порядок уменьшает число переменных при отладке и помогает быстро локализовать проблему по подсистемам.
