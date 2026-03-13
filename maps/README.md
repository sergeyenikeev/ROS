# Каталог карт PatrolBot

Этот каталог предназначен для рабочих карт, которые создаются во время реального
или тестового картографирования и затем используются для локализации и навигации.

Ожидаемые типовые файлы:

- `/home/ubuntu/ROS/maps/patrolbot_map.yaml`
- `/home/ubuntu/ROS/maps/patrolbot_map.pgm`

Как используется каталог:

- сюда пишет [save_map.sh](/home/ubuntu/ROS/scripts/save_map.sh);
- отсюда читает [run_localization.sh](/home/ubuntu/ROS/scripts/run_localization.sh);
- отсюда читает [run_navigation.sh](/home/ubuntu/ROS/scripts/run_navigation.sh);
- отсюда читает [run_patrol.sh](/home/ubuntu/ROS/scripts/run_patrol.sh).

Важно:

- это каталог для рабочих карт объекта;
- тестовая карта для integration-сценариев лежит отдельно в пакете `patrolbot_navigation/maps`;
- рабочие карты и тестовые ресурсы intentionally разделены, чтобы операторские
  артефакты не смешивались с версионируемыми файлами репозитория.
