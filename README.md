# PatrolBot

PatrolBot - практический проект мобильного робота для патрулирования помещений на базе Raspberry Pi, ROS 2 Jazzy, 2D LiDAR и дифференциального привода.

Репозиторий организован как ROS 2 workspace. Целевая директория развёртывания на Linux и Raspberry Pi: `/home/ubuntu/ROS`.

На текущем этапе уже подготовлены:
- каркас ROS 2 workspace и пакетная структура;
- описание робота в формате URDF/Xacro;
- базовые launch-профили для mock, hardware, SLAM, localization, navigation и patrol;
- удалённый `origin`, настроенный на `git@github.com:sergeyenikeev/ROS.git`.

Подробная документация находится в каталоге `docs/`.
