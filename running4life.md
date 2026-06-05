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

Si existe otro workspace con un paquete llamado `final_challenge` (por ejemplo
`~/ros2_ws`) ya cargado en la terminal, abrir una terminal limpia antes de
seguir. Después de hacer `source install/setup.bash` en este proyecto, validar:

```bash
ros2 pkg prefix final_challenge
```

Debe apuntar a:

```text
/home/karinam/MCR2_Final_Challenge/install/final_challenge
```

---

# 1. En la Jetson

## Terminal Jetson 1 — Cámara / ArUco base

```bash
ssh puzzlebot@10.42.162.217

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
ssh puzzlebot@10.42.162.217

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
ssh puzzlebot@10.42.162.217

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

colcon build --packages-select final_challenge
source install/setup.bash
ros2 pkg prefix final_challenge
```

## Comandos rápidos para correr el proyecto

Usa estos pasos desde la raíz del workspace para compilar y lanzar la pila física:

```bash
cd ~/MCR2_Final_Challenge
source /opt/ros/humble/setup.bash
unset RMW_IMPLEMENTATION
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

colcon build --packages-select final_challenge
source install/setup.bash

# Lanzar solo localización (sin navegación)
ros2 launch final_challenge physical_challenge.launch.py nav:=false use_rviz:=true --ros-args --log-level info

# Lanzar con navegación y RViz
ros2 launch final_challenge physical_challenge.launch.py nav:=true use_rviz:=true --ros-args --log-level info
```

---

# 3. Prueba 1 — Validar solo localización

Antes de activar navegación, correr sin `bug_controller`:

```bash
ros2 launch final_challenge physical_challenge.launch.py nav:=false use_rviz:=true
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

Objetivo: comprobar que el robot AVANZA hacia una meta simple en `+x` antes de
arriesgar la ruta completa del laberinto. Si en esta prueba el robot "da
vueltas" o no avanza, casi siempre es que el controlador NO recibe `/odom`, o
que `/odom` no se mueve porque no llegan los encoders (no es un waypoint malo).

## 6.1 Configurar la ruta

Editar `config/navigation_physical.yaml` con una ruta mínima (solo avanzar en `+x`):

```yaml
bug_controller:
  ros__parameters:
    loop: false
    waypoints_x: [0.60, 1.00]
    waypoints_y: [-0.28, -0.28]
```

Recordatorio del sistema de ejes (origen en la esquina inferior-izquierda):

```text
x positivo: hacia arriba del mapa
y negativo: hacia la derecha del mapa
theta0 = 0 apunta hacia +x
```

## 6.2 Cómo correr

Con la Jetson ya levantada (cámara/ArUco, micro-ROS Agent y LiDAR — pasos 1 a 3),
en la PC:

```bash
cd ~/MCR2_Final_Challenge
source /opt/ros/humble/setup.bash
unset RMW_IMPLEMENTATION
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

colcon build --packages-select final_challenge
source install/setup.bash

ros2 launch final_challenge physical_challenge.launch.py nav:=true use_rviz:=true
```

## 6.3 Verificar el pipeline ANTES de soltar el waypoint

Antes de confiar en el waypoint, confirma que la pose le llega al controlador y
que se mueve de verdad (esto descarta el síntoma de "dar vueltas"):

```bash
# 1) /odom publica a frecuencia estable (~50 Hz)
ros2 topic hz /odom

# 2) /odom CAMBIA al empujar el robot a mano (repite mientras lo mueves):
ros2 topic echo --once /odom    # x, y deben cambiar al moverlo

# 3) los encoders llegan (sin ellos no hay odom aunque lo demás esté bien)
ros2 topic hz /VelocityEncR
ros2 topic hz /VelocityEncL

# 4) el controlador está vivo y con la ruta cargada
ros2 param get /bug_controller waypoints_x
```

Si `/odom` NO cambia al empujar el robot, no sueltes el waypoint: arregla
primero encoders/localización (ver sección 10, Debug rápido).

## 6.4 Correr el waypoint y qué observar

Con el pipeline confirmado, en otra terminal:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /goal_reached
ros2 topic echo /odom --field pose.pose.position
```

Lo esperado:

```text
- /cmd_vel: linear.x positiva (avanza); angular.z pequeña (ajuste de rumbo).
- /odom: x sube de ~0.0 hacia ~1.0; y se mantiene cerca de -0.28.
- El robot avanza derecho hacia ARRIBA del mapa (+x); NO gira en el sitio.
- /goal_reached marca cada meta; con loop:false se detiene tras (1.00, -0.28).
```

Si el robot avanza pero hacia el lado equivocado (de lado o al revés), el
pipeline está bien y es un problema de SIGNO. Revisar en este orden:

```text
1. signo de theta (lo que reporta /odom vs la orientación real)
2. signo del bearing del ArUco
3. orientación física inicial del robot (debe mirar a +x)
4. sentido de los encoders (¿izquierda/derecha intercambiados?)
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

# 8. Prueba 6 — Navegación real

## 8.1 Opción A — Ruta corta SEGURA (primera prueba real, recomendada)

Ida y vuelta por el **pasillo izquierdo**, que está libre desde el inicio. El robot
avanza, pasa frente al marcador **70**, corrige su pose con el EKF, sigue pared si
hace falta, y regresa. Es navegación real, pero sin meterse a los cruces diagonales
que sabemos que chocan.

Ya viene activa en `config/navigation_physical.yaml`:

```yaml
bug_controller:
  ros__parameters:
    loop: false
    waypoints_x: [1.00, 1.70, 2.60, 1.70, 0.50]
    waypoints_y: [-0.28, -0.30, -0.20, -0.30, -0.28]
```

Recordatorio de ejes: `x` arriba, `y` negativo a la derecha, `theta0=0` mira a `+x`.

Antes de correr, confirma el pipeline (ver 6.3): `/odom` publica y cambia al mover el
robot, encoders llegan, `/scan` se ve en RViz. Luego:

```bash
# rebuild si cambiaste configs
cd ~/MCR2_Final_Challenge && colcon build --packages-select final_challenge && source install/setup.bash

ros2 launch final_challenge physical_challenge.launch.py nav:=true use_rviz:=true
```

En otra terminal:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /goal_reached
ros2 topic echo /odom --field pose.pose.position
ros2 topic echo /aruco/detections
```

Lo esperado:

```text
- El robot avanza hacia ARRIBA (+x) por el pasillo izquierdo (y se mantiene ~-0.3).
- Al pasar frente al ID 70, /aruco/detections publica [70, dist, bearing] y la pose
  en /odom se corrige (pequeño salto coherente con la posicion del marcador).
- Llega cerca del tope (x~2.6), da media vuelta y regresa hacia (0.50, -0.28).
- Con loop:false, al alcanzar el ultimo WP imprime 'Ruta completa' y se detiene.
- En RViz: la flecha de pose sigue el pasillo y el /scan dibuja las paredes a los lados.
```

Si roza una pared, baja `v_max`/`fw_linear_speed` en `navigation_physical.yaml` y revisa
que `following_walls_distance`/`front_stop_distance` sean conservadores.

## 8.2 Ruta completa del maze (solo tras validar 8.1)

Solo después de validar:

```text
/odom funciona y se corrige con ArUco
/scan se ve en RViz (paredes coherentes)
la ruta corta (8.1) se recorre sin chocar
```

cambiar a la **ruta completa** en `config/navigation_physical.yaml` (en el archivo está
comentado el lazo de 8 puntos `70->701->706->75->703->702->708->705`), poner `loop: true`,
**verificar cada tramo en RViz** contra el `/scan` y el mapa de marcadores (`viz_debug`),
y agregar puntos intermedios donde una recta roce pared. Luego correr igual que 8.1 y
revisar además:

```bash
ros2 topic echo /scan
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
