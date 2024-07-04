# ROS task

Практическое задание содержит три пакета, использующие основные интерфейсы ROS: ros_topic, ros_service, ros_action, а также пакет, который определяет кастомные сообщения ros_interfaces.

## Сборка
Скачайте и распакуйте архив в папку `~/ros2_ws/src`.

Сборка решения:

``` bash
cd ~/ros2_ws
colcon build
. install/setup.bash
```

## Решения

### ros_topic

Реализация простого подписчика и паблишера. Паблишер публикует текущее время в топик, подписчик вывод его в лог.

Запуск:

``` bash
# first terminal
ros2 run ros_topic talker

# second terminal
ros2 run ros_topic listener
```

### ros_service

Реализация простого сервиса. Использует следующее сообщение:

```
uint8 NOW = 0
uint8 DEFERRED = 1

uint8 type
float32 duration
---
bool success
string message
```

Если клиент отправляет запрос с типом NOW, то сервер немедленно возвращает успех. Если DEFERRED, то сервер засыпает на время duration, и после возвращает успех. Если в качестве типа клиент указывает другое число, сервер возвращает ошибку.

Запуск:

``` bash
# first terminal
ros2 run ros_service service

# second terminal
ros2 run ros_service client {type} {duration}
```

### ros_action

Реализация сервера действий. Использует следующее сообщение:

```
int32 target_position
float32 duration
---
std_msgs/Header header
string message
---
int32 position
```
Сервер хранит позицию, изначально равную нулю. Клиент в запросе указывает новую позицию и время, в течении которого сервер должен её достигнуть. В процессе движения сервер возвращает текущую позицию с частотой, которую можно настроить в файле [action_server.yaml](ros_action/config/action_server.yaml). После завершения движения сервер возвращает сообщение с результатом.

Запуск:

``` bash
# first terminal
ros2 launch ros_action action_server.launch.py

# second terminal
ros2 run ros_action client {target} {duration}
```
