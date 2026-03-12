# PatrolBot

PatrolBot - практический проект мобильного робота для патрулирования помещений на базе Raspberry Pi, ROS 2 Jazzy, 2D LiDAR и дифференциального привода.

Репозиторий организован как ROS 2 workspace. Целевая директория развёртывания на Linux и Raspberry Pi: `/home/ubuntu/ROS`.

На текущем этапе уже подготовлены:
- каркас ROS 2 workspace и пакетная структура;
- описание робота в формате URDF/Xacro;
- базовые launch-профили для mock, hardware, SLAM, localization, navigation и patrol;
- C++-узел базовой платформы с одометрией, watchdog по `cmd_vel` и публикацией диагностики;
- mock-драйвер моторов и mock-публикатор `LaserScan` для тестовой среды;
- подготовленные конфиги `slam_toolbox`, `AMCL` и `Nav2` с тестовой картой;
- интерфейсы патруля и C++-узел `mission_manager` с повторами, таймаутами и сервисами старта/остановки;
- unit-тесты для утилит, одометрии и state machine, а также integration launch-тесты с mock Nav2;
- удалённый `origin`, настроенный на `git@github.com:sergeyenikeev/ROS.git`.

Подробная документация находится в каталоге `docs/`.
