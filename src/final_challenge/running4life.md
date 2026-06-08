# Pruebas físicas Puzzlebot — Jetson + PC

## Objetivo

1. Levantar sensores y comunicación en Jetson.
2. Validar ArUco, encoders y LiDAR.
3. Probar localización sin navegación.
4. Validar parámetros:
   * `marker_size_m = 0.094`
   * `use_tvec_z_correction = false`
   * pose inicial correcta
   * marcadores correctos
5. Probar un waypoint simple.
6. Probar navegación completa.

---

# 0. Configuración común

En **todas** las terminales:

```bash
source /opt/ros/humble/setup.bash
unset RMW_IMPLEMENTATION
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

Validar que el workspace correcto esté cargado:

```bash
ros2 pkg prefix final_challenge
# debe apuntar a: /home/karinam/MCR2_Final_Challenge/install/final_challenge
```

---

# 1. En la Jetson — Sensores base

## Terminal Jetson 1 — Cámara / ArUco

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

Verificar:

```bash
ros2 topic list | grep image
ros2 topic hz /video_source/raw
```

## Terminal Jetson 2 — micro-ROS Agent (encoders)

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

Verificar:

```bash
ros2 topic hz /VelocityEncR
ros2 topic hz /VelocityEncL
```

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

Verificar:

```bash
ros2 topic hz /scan
```

Si no aparece `/scan`, probar `/dev/ttyUSB0`.

---

# 2. Build del proyecto

## En la PC

```bash
cd ~/MCR2_Final_Challenge
source /opt/ros/humble/setup.bash
unset RMW_IMPLEMENTATION
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

colcon build --packages-select final_challenge
source install/setup.bash
```

## En la Jetson (fix de lag del LiDAR)

El `/scan` es pesado. Si `bug_controller` corre en la PC, cada mensaje viaja
por WiFi causando lag. La solución es correr los nodos que usan el scan
directamente en la Jetson.

Sincronizar el proyecto a la Jetson (desde la PC):

```bash
rsync -av ~/MCR2_Final_Challenge/ puzzlebot@10.42.162.217:~/MCR2_Final_Challenge/
```

Build en la Jetson:

```bash
ssh puzzlebot@10.42.162.217
cd ~/MCR2_Final_Challenge
source /opt/ros/humble/setup.bash
unset RMW_IMPLEMENTATION
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
colcon build --packages-select final_challenge
source install/setup.bash
```

---

# 3. Cómo lanzar

## Opción A — Todo en una máquina (pruebas rápidas)

```bash
# Solo localización
ros2 launch final_challenge physical_challenge.launch.py nav:=false use_rviz:=true

# Con navegación
ros2 launch final_challenge physical_challenge.launch.py nav:=true use_rviz:=true
```

## Opción B — Jetson + PC separados (recomendado, sin lag de LiDAR)

**Terminal Jetson 4** — navegación sin RViz:

```bash
ssh puzzlebot@10.42.162.217
cd ~/MCR2_Final_Challenge
source /opt/ros/humble/setup.bash
unset RMW_IMPLEMENTATION
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source install/setup.bash

# Con navegación (normal)
ros2 launch final_challenge physical_jetson.launch.py

# Solo localización (para validar sin mover el robot)
ros2 launch final_challenge physical_jetson.launch.py nav:=false
```

**PC** — solo RViz:

```bash
cd ~/MCR2_Final_Challenge
source /opt/ros/humble/setup.bash
unset RMW_IMPLEMENTATION
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source install/setup.bash

ros2 launch final_challenge physical_pc.launch.py
```

> Si RViz sigue lento, reducir frecuencia del scan para visualización:
> ```bash
> ros2 run topic_tools throttle messages /scan 5.0 /scan_viz
> ```
> Luego suscribirse a `/scan_viz` en RViz.

---

# 4. Prueba 1 — Validar localización

Con `nav:=false` corriendo, revisar:

```bash
ros2 topic echo /odom
ros2 topic echo /aruco/detections
ros2 topic hz /odom
ros2 topic hz /aruco/detections
```

Esperado:

```text
/odom cambia al mover el robot.
/aruco/detections aparece cuando la cámara ve un marcador.
tvec.z no está inflado.
/scan existe con frecuencia estable.
RViz muestra robot, odom, marcadores y debug de ArUco.
```

---

# 5. Prueba 2 — Validar parámetros

```bash
ros2 param get /aruco_detector marker_size_m          # → 0.094
ros2 param get /aruco_detector use_tvec_z_correction  # → False

ros2 param get /localisation x0      # → 0.325
ros2 param get /localisation y0      # → -0.28
ros2 param get /localisation theta0  # → 0.0

ros2 param get /localisation marker_ids
ros2 param get /localisation marker_pos_x
ros2 param get /localisation marker_pos_y
```

Esperado marcadores:

```yaml
marker_ids:   [70, 75, 701, 702, 703, 705, 706, 708]
marker_pos_x: [1.85, 2.75, 2.82, 0.27, 1.24, 0.89, 2.455, 1.185]
marker_pos_y: [-0.30, -2.40, 0.00, -1.83, -2.07, -1.20, -1.255, -1.21]
```

---

# 6. Prueba 3 — Validar distancia ArUco

Poner el robot frente al ArUco 70 a 26 cm:

```bash
ros2 topic echo /aruco/detections
```

Esperado: `tvec.z ≈ 0.26 m`. Antes salía ~0.38 m porque `marker_size_m` estaba mal.

---

# 7. Prueba 4 — Waypoint simple

## Configurar `config/navigation_physical.yaml`

```yaml
bug_controller:
  ros__parameters:
    loop: false
    waypoints_x: [0.60, 1.00]
    waypoints_y: [-0.28, -0.28]
```

Sistema de ejes:

```text
x positivo → arriba del mapa
y negativo → derecha del mapa
theta0 = 0 → mira hacia +x
```

## Sincronizar y correr (Opción B)

```bash
# Desde la PC
rsync -av ~/MCR2_Final_Challenge/ puzzlebot@10.42.162.217:~/MCR2_Final_Challenge/

# En la Jetson
cd ~/MCR2_Final_Challenge
colcon build --packages-select final_challenge && source install/setup.bash
ros2 launch final_challenge physical_jetson.launch.py

# En la PC
ros2 launch final_challenge physical_pc.launch.py
```

## Verificar pipeline ANTES de soltar

```bash
ros2 topic hz /odom                          # ~50 Hz estable
ros2 topic echo --once /odom                 # x,y cambian al mover el robot
ros2 topic hz /VelocityEncR                  # encoders llegan
ros2 param get /bug_controller waypoints_x   # ruta cargada
```

## Qué observar

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /goal_reached
ros2 topic echo /odom --field pose.pose.position
```

Esperado:

```text
- /cmd_vel: linear.x positiva, angular.z pequeña.
- /odom: x sube hacia ~1.0, y se mantiene cerca de -0.28.
- El robot avanza hacia +x sin girar en el sitio.
- /goal_reached publica al llegar; con loop:false se detiene.
```

---

# 8. Prueba 5 — Movimiento hacia la derecha

```yaml
waypoints_x: [0.60, 0.60]
waypoints_y: [-0.50, -1.00]
loop: false
```

Esperado: `y` se vuelve más negativo en `/odom`.

---

# 9. Prueba 6 — Navegación real

## Opción A — Ruta corta SEGURA (recomendada primero)

```yaml
bug_controller:
  ros__parameters:
    loop: false
    waypoints_x: [1.00, 1.70, 2.60, 1.70, 0.50]
    waypoints_y: [-0.28, -0.30, -0.20, -0.30, -0.28]
```

Sincronizar, rebuild en Jetson y lanzar (Opción B). Monitorear:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /goal_reached
ros2 topic echo /odom --field pose.pose.position
ros2 topic echo /aruco/detections
```

Esperado:

```text
- Avanza hacia +x por el pasillo izquierdo (y ~-0.3).
- Al ver ID 70, /aruco/detections publica y /odom se corrige.
- Llega a x~2.6, da media vuelta, regresa a (0.50, -0.28).
- Con loop:false imprime 'Ruta completa' y se detiene.
```

## Opción B — Ruta completa del maze

Solo tras validar la ruta corta sin chocar. Activar el lazo de 8 puntos
(`70→701→706→75→703→702→708→705`) con `loop: true` y verificar cada tramo
en RViz antes de soltar el robot.

---

# 10. rqt_image_view

```bash
cd ~/MCR2_Final_Challenge
source /opt/ros/humble/setup.bash
unset RMW_IMPLEMENTATION
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
source install/setup.bash

ros2 run rqt_image_view rqt_image_view
```

---

# 11. Debug rápido

## No hay `/odom`
```bash
ros2 topic echo /VelocityEncR
ros2 topic echo /VelocityEncL
```
Si no llegan → revisar micro-ROS Agent.

## No hay `/aruco/detections`
```bash
ros2 topic list | grep image
ros2 topic hz /video_source/raw
ros2 node list
```

## No hay `/scan`
```bash
ros2 topic hz /scan
```
Probar `/dev/ttyUSB0` o `/dev/ttyUSB1`.

## `/cmd_vel` en cero
```bash
ros2 topic echo /cmd_vel
ros2 topic echo /goal_reached
ros2 param list /bug_controller
```
Causas: waypoints vacíos, odom no llega, scan no llega, goal ya alcanzado.

## `/cmd_vel` con velocidad pero robot inmóvil
Causas: motores, micro-ROS no recibe cmd_vel, batería baja, remapeo incorrecto.

---

# 12. CHRONY — sincronización de tiempo

## PC
```bash
sudo apt update && sudo apt install chrony
sudo systemctl enable chrony && sudo systemctl start chrony
sudo nano /etc/chrony/chrony.conf
sudo systemctl restart chrony
chronyc tracking
```

## Jetson
```bash
systemctl status systemd-timesyncd
sudo nano /etc/systemd/timesyncd.conf
sudo systemctl restart systemd-timesyncd
sudo timedatectl set-ntp true
timedatectl status
```

---

# 13. Orden recomendado final

```text
1.  Jetson T1: cámara / ArUco      (ros2 launch puzzlebot_ros aruco_jetson.launch.py)
2.  Jetson T2: micro-ROS Agent     (ros2 launch puzzlebot_ros micro_ros_agent.launch.py)
3.  Jetson T3: LiDAR               (ros2 launch rplidar_ros rplidar_a1_launch.py ...)
4.  PC: rsync del proyecto         (rsync -av ~/MCR2_Final_Challenge/ puzzlebot@10.42.162.217:...)
5.  Jetson: colcon build
6.  Jetson T4: physical_jetson.launch.py nav:=false   → validar primero sin mover
7.  PC: physical_pc.launch.py
8.  Validar parámetros (/aruco_detector, /localisation)
9.  Validar /odom cambia al mover el robot
10. Validar /aruco/detections
11. Validar /scan en RViz
12. Jetson T4: physical_jetson.launch.py nav:=true    → waypoint simple +x
13. Probar movimiento hacia -y
14. Probar navegación completa
```
