#!/usr/bin/env python3
"""Genera worlds/maze.world a norma: 3x3 m, pasillos 60 cm, paredes 2 cm.

Reproduce el trazado del boceto (8 ArUcos: 70,706,75,701,703,705,708,702).
Las paredes se definen como segmentos (linea central) y se emiten como cajas.
Los ArUcos se montan sobre la cara de su pared, mirando hacia el pasillo.

Facing (yaw): 0=+x(este), pi/2=+y(norte), pi=-x(oeste), -pi/2=-y(sur).
"""
import math

TH = 0.02     # grosor de pared (2 cm)
H = 0.30      # alto de pared (>=25 cm)

# --- Paredes: segmentos por linea central (x1,y1,x2,y2) ---------------------
# Caja de 3x3 m centrada en el origen (lineas de reja: -1.5,-0.9,-0.3,0.3,0.9,1.5)
walls = [
    # Perimetro exterior (con hueco/entrada arriba-derecha: x de 0.6 a 1.5)
    (-1.5, -1.5,  1.5, -1.5),   # abajo
    (-1.5, -1.5, -1.5,  1.5),   # izquierda
    ( 1.5, -1.5,  1.5,  1.5),   # derecha
    (-1.5,  1.5,  0.6,  1.5),   # arriba (deja entrada 0.6..1.5)
    # Interiores
    ( 0.6,  1.5,  0.6,  0.6),   # A: pared colgante (75)
    (-0.9,  0.6, -0.15, 0.6),   # B: base del bracket de 706
    (-0.9,  1.5, -0.9,  0.6),   # C: lado izq bracket
    (-0.15, 1.5, -0.15, 0.6),   # D: lado der bracket
    (-1.5,  0.3, -0.6,  0.3),   # E: pared de 70
    (-0.3,  0.0, -0.3, -0.9),   # F: vertical central (708/705)
    (-0.6, -0.15, 0.3, -0.15),  # G: horizontal central
    ( 0.3,  0.6,  0.3, -0.3),   # H: vertical centro-derecha
    ( 0.6, -0.15, 1.2, -0.15),  # I: pared de 703
    ( 0.6, -0.3,  0.6, -0.9),   # J
    (-0.3, -0.9,  0.6, -0.9),   # K: pared de 702
    (-0.6, -1.5, -0.6, -0.9),   # L: abajo-izquierda
]

# --- ArUcos: (id, x, y, yaw_facing) -----------------------------------------
# Posicion = sobre la cara de la pared, mirando al pasillo.
arucos = [
    (75,   0.57,  1.05,  math.pi),       # oeste
    (706, -0.50,  0.57, -math.pi/2),     # sur
    (70,  -0.85,  0.27, -math.pi/2),     # sur
    (701, -1.46, -0.10,  0.0),           # este (cara interior pared izq)
    (708, -0.27, -0.10,  0.0),           # este
    (705, -0.33, -0.40,  math.pi),       # oeste
    (703,  0.63, -0.18,  math.pi/2),     # norte
    (702,  0.40, -0.87,  0.0),           # este
]

def wall_sdf(i, x1, y1, x2, y2):
    cx, cy = (x1+x2)/2.0, (y1+y2)/2.0
    L = math.hypot(x2-x1, y2-y1) + TH    # +TH para cerrar esquinas
    yaw = math.atan2(y2-y1, x2-x1)
    return f"""    <link name='Wall_{i}'>
      <pose>{cx:.4f} {cy:.4f} 0 0 0 {yaw:.5f}</pose>
      <collision name='Wall_{i}_C'>
        <pose>0 0 {H/2:.3f} 0 0 0</pose>
        <geometry><box><size>{L:.4f} {TH} {H}</size></box></geometry>
      </collision>
      <visual name='Wall_{i}_V'>
        <pose>0 0 {H/2:.3f} 0 0 0</pose>
        <geometry><box><size>{L:.4f} {TH} {H}</size></box></geometry>
        <material>
          <ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse>
        </material>
      </visual>
    </link>
"""

def aruco_sdf(mid, x, y, yaw):
    z = 0.15
    return f"""    <model name='aruco_{mid}'>
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z} 0 0 {yaw:.5f}</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>0.02 0.18 0.18</size></box></geometry>
        </collision>
        <visual name='blue_backplate'>
          <cast_shadows>0</cast_shadows>
          <geometry><box><size>0.02 0.18 0.18</size></box></geometry>
          <material><ambient>0 0.15 1 1</ambient><diffuse>0 0.15 1 1</diffuse>
            <emissive>0 0.05 0.4 1</emissive></material>
        </visual>
        <visual name='aruco_texture'>
          <cast_shadows>0</cast_shadows>
          <pose>0.011 0 0 1.5708 0 1.5708</pose>
          <geometry><plane><normal>0 0 1</normal><size>0.15 0.15</size></plane></geometry>
          <material>
            <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular><emissive>0.3 0.3 0.3 1</emissive>
            <pbr><metal><albedo_map>file://worlds/aruco_textures/aruco_{mid}.png</albedo_map>
              <metalness>0.0</metalness><roughness>1.0</roughness></metal></pbr>
          </material>
        </visual>
      </link>
    </model>
"""

HEADER = """<sdf version='1.7'>
  <world name='default'>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name='visual'><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual>
      </link>
    </model>
    <gui fullscreen='0'><camera name='user_camera'><pose>0 0 6 0 1.5708 0</pose><view_controller>orbit</view_controller></camera></gui>
"""

def main():
    parts = [HEADER]
    parts.append("    <model name='mapcorreg'>\n      <static>1</static>\n")
    for i, seg in enumerate(walls):
        parts.append(wall_sdf(i, *seg))
    parts.append("    </model>\n")
    parts.append("    <!-- simulated_aruco_markers_start -->\n")
    for a in arucos:
        parts.append(aruco_sdf(*a))
    parts.append("  </world>\n</sdf>\n")
    with open('worlds/maze.world', 'w') as f:
        f.write("".join(parts))
    print(f"maze.world generado: {len(walls)} paredes, {len(arucos)} arucos, 3x3 m, pasillos 0.6 m, grosor {TH} m")

if __name__ == '__main__':
    main()
