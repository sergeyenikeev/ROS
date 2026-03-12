# Временная метка публикации статуса патруля.
builtin_interfaces/Time stamp

# Имя текущей миссии.
string mission_name

# Текущее состояние state machine.
string state

# Флаг активности миссии.
bool active

# Индекс текущей точки маршрута.
uint32 current_waypoint_index

# Имя текущей точки маршрута.
string current_waypoint_name

# Количество успешно завершённых точек.
uint32 completed_waypoints

# Количество точек, завершившихся ошибкой.
uint32 failed_waypoints

# Количество завершённых циклов маршрута.
uint32 loop_count

# Количество использованных повторов для текущей точки.
uint32 retry_count_for_current_waypoint

# Последняя текстовая ошибка.
string last_error
