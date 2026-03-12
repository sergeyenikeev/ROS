# Настройка Raspberry Pi для PatrolBot

## 1. Базовые предпосылки

Целевая конфигурация:

- Raspberry Pi 4 или 5;
- карта памяти или SSD;
- Ubuntu Server 24.04 LTS arm64;
- сеть с доступом в интернет;
- пользователь `ubuntu`;
- проект в `/home/ubuntu/ROS`.

## 2. Подготовка ОС

1. Записать Ubuntu Server 24.04 LTS arm64 на карту памяти.
2. Включить SSH через Raspberry Pi Imager или вручную.
3. Задать имя хоста, например `patrolbot`.
4. Подключить Raspberry Pi к сети и войти по SSH.

Проверка:

```bash
uname -m
lsb_release -a
hostname
```

Ожидаемо:

- `aarch64`
- Ubuntu 24.04
- корректное имя хоста

## 3. Размещение репозитория

Создать каталог и клонировать проект:

```bash
mkdir -p /home/ubuntu/ROS
git clone git@github.com:sergeyenikeev/ROS.git /home/ubuntu/ROS
```

Если SSH ещё не настроен:

```bash
mkdir -p /home/ubuntu/ROS
git clone https://github.com/sergeyenikeev/ROS.git /home/ubuntu/ROS
```

## 4. Установка зависимостей

Запустить готовый скрипт:

```bash
/home/ubuntu/ROS/scripts/install_dependencies.sh
```

Что делает скрипт:

- настраивает репозиторий ROS 2;
- ставит ROS 2 Jazzy;
- ставит `colcon`, `rosdep`, `clang-format`;
- подтягивает зависимости workspace через `rosdep`.

## 5. Сборка проекта

```bash
/home/ubuntu/ROS/scripts/build_workspace.sh
```

После сборки подключить окружение:

```bash
source /home/ubuntu/ROS/scripts/source_workspace.sh
```

## 6. Первый запуск без железа

Проверка mock-профиля:

```bash
source /home/ubuntu/ROS/scripts/source_workspace.sh
ros2 launch patrolbot_bringup mock_bringup.launch.py
```

Нужно убедиться:

- есть `/scan`;
- есть `/odom`;
- есть `/diagnostics`;
- есть TF `odom -> base_footprint`.

## 7. Подключение реального LiDAR

Проект ожидает внешний ROS 2 драйвер, публикующий:

- топик `/scan`
- корректный `frame_id`, совместимый с `laser_frame`

Порядок:

1. Подключить LiDAR.
2. Установить его драйвер.
3. Проверить `/scan`:

```bash
source /home/ubuntu/ROS/scripts/source_workspace.sh
ros2 topic echo /scan --once
```

## 8. Подключение моторного контроллера

В текущей версии `driver_mode=serial` является заготовкой. Нужно:

1. определить реальный протокол контроллера;
2. доработать `SerialMotorDriver`;
3. проверить порт:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

4. обновить:

- `/home/ubuntu/ROS/src/patrolbot_base/config/base_hardware.yaml`

## 9. Первый запуск на железе

После подключения LiDAR и адаптации драйвера базы:

```bash
source /home/ubuntu/ROS/scripts/source_workspace.sh
ros2 launch patrolbot_bringup hardware_bringup.launch.py
```

Проверить:

- публикацию `/odom`;
- обновление `joint_states`;
- наличие `/scan`;
- отсутствие ошибок в `/diagnostics`.

## 10. Типовые ошибки

### `rosdep install` не находит пакеты

- проверить интернет;
- убедиться, что добавлен репозиторий ROS 2;
- повторить `rosdep update`.

### `colcon build` падает на пользовательских интерфейсах

- проверить, что установлен `ros-jazzy-yaml-cpp-vendor`;
- убедиться, что подключён `/opt/ros/jazzy/setup.bash`.

### Нет доступа к serial-порту

- добавить пользователя в группу `dialout`:

```bash
sudo usermod -a -G dialout ubuntu
```

- перелогиниться.

### Падает `AMCL` или `Nav2`

- проверить карту;
- проверить наличие TF `map`, `odom`, `base_footprint`;
- проверить согласованность `frame_id` в конфиге LiDAR и URDF.
