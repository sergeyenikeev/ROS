# Список функций PatrolBot

- [В РАБОТЕ] Базовый каркас ROS 2 workspace и пакетная структура проекта.
- [СДЕЛАНО] Описание робота в `URDF/Xacro` и базовая TF-цепочка `map -> odom -> base_footprint -> base_link -> laser_frame`.
- [СДЕЛАНО] Базовые launch-файлы `mock_bringup`, `hardware_bringup`, `slam`, `localization`, `navigation`, `patrol`.
- [СДЕЛАНО] Базовая платформа с ограничением скоростей, watchdog по `cmd_vel`, одометрией и публикацией `diagnostic_msgs/DiagnosticArray`.
- [СДЕЛАНО] Mock-драйвер моторов и mock-публикатор `LaserScan` для запуска без реального железа.
- [ПЛАНИРУЕТСЯ] SLAM, локализация, Nav2 и логика патрулирования.
