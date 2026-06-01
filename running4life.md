# Pruebas físicas Puzzlebot — Jetson + PC

## Objetivo

Probar el sistema por etapas:

1. Levantar sensores y comunicación en Jetson.
2. Validar ArUco, encoders y LiDAR.
3. Probar localización sin navegación.
4. Validar parámetros corregidos:

   * `marker_size_m = 0.094`
   * `use_tvec_z_correction = false`
   * pose inicial correcta
   * marcadores correctos
5. Probar un waypoint simple.
6. Probar navegación completa.

---

# 0. Configuración común

En todas las terminales usar:

```bash
source /opt/ros/humble/setup.bash
unset RMW_IMPLEMENTATION
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

Si existe otro workspace con un paquete llamado `puzzlebot_sim2` (por ejemplo
`~/ros2_ws`) ya cargado en la terminal, abrir una terminal limpia antes de
seguir. Después de hacer `source install/setup.bash` en este proyecto, validar:

```bash
ros2 pkg prefix puzzlebot_sim2
```

Debe apuntar a:

```text
/home/karinam/MCR2_Final_Challenge/install/puzzlebot_sim2
```

---

# 1. En la Jetson

## Terminal Jetson 1 — Cámara / ArUco base

```bash
ssh puzzlebot@10.201.233.217

```

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
unset RMW_IMPLEMENTATION
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source install/setup.bash

ros2 launch puzzlebot_ros aruco_jetson.launch.py
```

Verificar que la cámara esté activa:

```bash
ros2 topic list | grep image
ros2 topic hz /video_source/raw
```

Si el tópico tiene otro nombre, revisar:

```bash
ros2 topic list
```

---

## Terminal Jetson 2 — micro-ROS Agent

```bash
ssh puzzlebot@10.201.233.217

```

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
unset RMW_IMPLEMENTATION
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source install/setup.bash

ros2 launch puzzlebot_ros micro_ros_agent.launch.py
```

Verificar encoders:

```bash
ros2 topic list | grep Velocity
ros2 topic echo /VelocityEncR
ros2 topic echo /VelocityEncL
```

También revisar frecuencia:

```bash
ros2 topic hz /VelocityEncR
ros2 topic hz /VelocityEncL
```

---

## Terminal Jetson 3 — LiDAR

```bash
ssh puzzlebot@10.201.233.217

```

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
unset RMW_IMPLEMENTATION
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source install/setup.bash

ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB1
```

Verificar LiDAR:

```bash
ros2 topic echo /scan
ros2 topic hz /scan
```

Si no aparece `/scan`, probar el otro puerto:

```bash
ros2 launch rplidar_ros rplidar_lidar_a1_launch.py serial_port:=/dev/ttyUSB0
```

---

# 2. En la PC o en la Jetson donde esté MCR2_Final_Challenge

## Terminal 4 — Build del proyecto

```bash
cd ~/MCR2_Final_Challenge
source /opt/ros/humble/setup.bash
unset RMW_IMPLEMENTATION
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

colcon build --packages-select puzzlebot_sim2
source install/setup.bash
ros2 pkg prefix puzzlebot_sim2
```

---

# 3. Prueba 1 — Validar solo localización

Antes de activar navegación, correr sin `bug_controller`:

```bash
ros2 launch puzzlebot_sim2 physical_challenge.launch.py nav:=false use_rviz:=true
```

En otra terminal:

```bash
cd ~/MCR2_Final_Challenge
source /opt/ros/humble/setup.bash
unset RMW_IMPLEMENTATION
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source install/setup.bash
```

Revisar tópicos:

```bash
ros2 topic echo /odom
ros2 topic echo /aruco/detections
ros2 topic hz /odom
ros2 topic hz /aruco/detections
```

Lo esperado:

```text
/odom cambia cuando se mueve el robot.
/aruco/detections aparece cuando la cámara ve un marcador.
tvec.z ya no está inflado.
/scan existe y tiene frecuencia estable.
RViz muestra robot, odom, marcadores y debug de ArUco.
```

---

# 4. Prueba 2 — Validar parámetros cargados

Con el launch corriendo, revisar:

```bash
ros2 param get /aruco_detector marker_size_m
ros2 param get /aruco_detector use_tvec_z_correction
```

Esperado:

```text
marker_size_m = 0.094
use_tvec_z_correction = False
```

Si `marker_size_m` aparece como `0.14`, entonces no está cargando `localisation_physical.yaml`.

También revisar localización:

```bash
ros2 param get /localisation x0
ros2 param get /localisation y0
ros2 param get /localisation theta0
```

Esperado:

```text
x0 = 0.325
y0 = -0.28
theta0 = 0.0
```

Revisar marcadores:

```bash
ros2 param get /localisation marker_ids
ros2 param get /localisation marker_pos_x
ros2 param get /localisation marker_pos_y
```

Esperado:

```yaml
marker_ids:   [70, 75, 701, 702, 703, 705, 706, 708]
marker_pos_x: [1.85, 2.75, 2.82, 0.27, 1.24, 0.89, 2.455, 1.185]
marker_pos_y: [-0.30, -2.40, 0.00, -1.83, -2.07, -1.20, -1.255, -1.21]
```

---

# 5. Prueba 3 — Validar distancia ArUco

Poner el robot frente al ArUco 70 a una distancia conocida, por ejemplo 26 cm.

Revisar:

```bash
ros2 topic echo /aruco/detections
```

Esperado:

```text
Si el robot está a 26 cm del ArUco 70, tvec.z debe estar cerca de 0.26 m.
Antes salía cerca de 0.38 m porque marker_size_m estaba mal.
```

Si ahora sale cercano a la distancia real, el cambio de:

```text
marker_size_m: 0.094
```

está funcionando.

---

# 6. Prueba 4 — Waypoint simple

Antes de probar todo el laberinto, poner una ruta muy simple en:

```bash
config/navigation_physical.yaml
```

Usar:

```yaml
waypoints_x: [0.60, 1.00]
waypoints_y: [-0.28, -0.28]
loop: false
```

Esto solo pide avanzar hacia `+x`.

Recordatorio del nuevo sistema de ejes:

```text
x positivo: hacia arriba del mapa
y negativo: hacia la derecha del mapa
theta0 = 0 apunta hacia +x
```

Ejecutar navegación:

```bash
ros2 launch puzzlebot_sim2 physical_challenge.launch.py use_rviz:=true
```

En otra terminal revisar:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /goal_reached
ros2 topic echo /odom
```

Lo esperado:

```text
El robot debe avanzar hacia arriba del mapa.
odom debe aumentar principalmente en x.
cmd_vel debe publicar velocidades.
El robot no debe irse hacia el lado contrario.
```

Si el robot avanza al revés o de lado, revisar:

```text
signo de theta
signo del bearing
orientación inicial física del robot
sentido de encoders
```

---

# 7. Prueba 5 — Movimiento hacia la derecha del mapa

Para probar el eje `y`, usar una ruta simple donde `y` se haga más negativo:

```yaml
waypoints_x: [0.60, 0.60]
waypoints_y: [-0.50, -1.00]
loop: false
```

Esperado:

```text
El robot debe moverse hacia la derecha del mapa.
En odom, y debe volverse más negativo.
```

---

# 8. Prueba 6 — Navegación completa

Solo después de validar:

```text
/odom funciona
/aruco/detections funciona
/scan funciona
marker_size_m = 0.094
use_tvec_z_correction = False
x0, y0, theta0 están correctos
el waypoint hacia +x funciona
el movimiento hacia -y funciona
```

activar la ruta completa del laberinto en:

```bash
config/navigation_physical.yaml
```

y correr:

```bash
ros2 launch puzzlebot_sim2 physical_challenge.launch.py use_rviz:=true
```

Revisar:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /goal_reached
ros2 topic echo /odom
ros2 topic echo /scan
ros2 topic echo /aruco/detections
```

---

# 9. rqt_image_view

Para ver la cámara:

```bash
cd ~/MCR2_Final_Challenge
source /opt/ros/humble/setup.bash
unset RMW_IMPLEMENTATION
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source install/setup.bash

ros2 run rqt_image_view rqt_image_view
```

Seleccionar el tópico de imagen correspondiente.

---

# 10. Debug rápido

## Si no hay `/odom`

Revisar encoders:

```bash
ros2 topic echo /VelocityEncR
ros2 topic echo /VelocityEncL
```

Si no llegan, revisar micro-ROS Agent.

---

## Si no hay `/aruco/detections`

Revisar cámara:

```bash
ros2 topic list | grep image
ros2 topic hz /video_source/raw
```

Revisar que el detector esté corriendo:

```bash
ros2 node list
ros2 param list /aruco_detector
```

---

## Si no hay `/scan`

Revisar LiDAR:

```bash
ros2 topic hz /scan
```

Probar otro puerto:

```bash
/dev/ttyUSB0
/dev/ttyUSB1
```

---

## Si `/cmd_vel` está en cero

Posibles causas:

```text
waypoints vacíos
bug_controller no está activo
odom no llega
scan no llega
el goal ya se considera alcanzado
```

Revisar:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /goal_reached
ros2 param list /bug_controller
```

---

## Si `/cmd_vel` manda velocidad pero el robot no se mueve

Posibles causas:

```text
problema en motores
micro-ROS no recibe cmd_vel
driver físico no está activo
batería baja
remapeo incorrecto
```

---

# 11. CHRONY / sincronización de tiempo

## En la PC

```bash
hostname -I

sudo apt update
sudo apt install chrony
sudo systemctl enable chrony
sudo systemctl start chrony

sudo nano /etc/chrony/chrony.conf
```

Después:

```bash
sudo systemctl restart chrony
sudo ss -lunp | grep :123
chronyc tracking
```

---

## En la Jetson

Revisar estado:

```bash
systemctl status systemd-timesyncd
```

Editar:

```bash
sudo nano /etc/systemd/timesyncd.conf
```

Reiniciar:

```bash
sudo systemctl restart systemd-timesyncd
sudo timedatectl set-ntp true
timedatectl status
```

---

# 12. Orden recomendado final

```text
1. Jetson Terminal 1: cámara / ArUco.
2. Jetson Terminal 2: micro-ROS Agent.
3. Jetson Terminal 3: LiDAR.
4. Build del proyecto MCR2_Final_Challenge.
5. Launch con nav:=false.
6. Validar parámetros.
7. Validar /odom.
8. Validar /aruco/detections.
9. Validar /scan.
10. Probar waypoint simple hacia +x.
11. Probar movimiento hacia -y.
12. Probar navegación completa.
```
