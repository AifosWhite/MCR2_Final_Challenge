#!/usr/bin/env python3
"""
viz_debug.py  -  Visualizacion y logging de depuracion para el Puzzlebot FISICO.

Pensado para tunear la localizacion (EKF + ArUco) viendo en RViz y en consola:
  - El MAPA de marcadores (donde estan los ArUcos segun tu config) con su ID.
  - La POSE estimada del robot (donde cree que esta) + su trayectoria.
  - Cuando VE un ArUco: una linea desde el robot a DONDE esta leyendo el marcador
    (la medicion dist/bearing). Comparar esa lectura con el cubo del mapa = el
    error de localizacion de un vistazo.

No tiene dependencias de simulacion. Publica MarkerArray en /localisation_markers
(el topico que ya escucha tu RViz) y loguea de forma compacta SOLO cuando el
error de localizacion supera 'err_log_threshold' (default 0.15 m):
    aruco 70 | error 0.32 m (leo en -1.20,+0.15 | mapa -1.25,+0.10)

Suscribe:
  /odom               (nav_msgs/Odometry)        -> pose estimada
  /aruco/detections   (std_msgs/Float32MultiArray = [id, distance, bearing])
"""

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import ColorRGBA, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


class VizDebug(Node):
    def __init__(self):
        super().__init__('viz_debug')

        # Mapa de marcadores (mismas posiciones que localisation_physical.yaml)
        # Origen (0,0) = esquina inferior-izquierda. x crece arriba, y negativo derecha (m).
        self.declare_parameter('marker_ids', [70, 75, 701, 702, 703, 705, 706, 708])
        self.declare_parameter('marker_pos_x',
                               [1.85, 2.75, 2.82, 0.27, 1.24, 0.89, 2.455, 1.185])
        self.declare_parameter('marker_pos_y',
                               [-0.30, -2.40, 0.00, -1.83, -2.07, -1.20, -1.255, -1.21])
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('detections_topic', '/aruco/detections')
        self.declare_parameter('frame_id', 'odom')
        # Solo loguea en consola cuando el error de localizacion supere esto (m).
        self.declare_parameter('err_log_threshold', 0.15)

        ids = list(self.get_parameter('marker_ids').value)
        xs = list(self.get_parameter('marker_pos_x').value)
        ys = list(self.get_parameter('marker_pos_y').value)
        self.markers_map = {int(i): (float(x), float(y))
                            for i, x, y in zip(ids, xs, ys)}
        self.frame_id = str(self.get_parameter('frame_id').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        det_topic = str(self.get_parameter('detections_topic').value)
        self.err_log_threshold = float(
            self.get_parameter('err_log_threshold').value)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.cov = (0.0, 0.0, 0.0)      # (Pxx, Pyy, Pxy) de la covarianza del EKF
        self.path = []                  # trayectoria (x,y)
        self.last_obs = None            # (id, ox, oy) ultima lectura de aruco
        self.last_obs_count = 0         # ciclos que mantenemos la linea visible

        self.create_subscription(Odometry, odom_topic, self.odom_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, det_topic, self.det_cb, 10)
        self.pub = self.create_publisher(MarkerArray, '/localisation_markers', 10)
        self.create_timer(0.1, self.publish_markers)

        self.get_logger().info(
            f'viz_debug listo: {len(self.markers_map)} marcadores en el mapa, '
            f'publica /localisation_markers en frame "{self.frame_id}".')

    # ---- Callbacks --------------------------------------------------------
    def odom_cb(self, msg):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = float(np.arctan2(siny, cosy))
        # Covarianza de pose: indices 0=xx, 7=yy, 1=xy del array 6x6 de Odometry.
        cov = msg.pose.covariance
        self.cov = (float(cov[0]), float(cov[7]), float(cov[1]))
        self.path.append((self.x, self.y))
        if len(self.path) > 2000:
            self.path = self.path[-2000:]

    def det_cb(self, msg):
        if len(msg.data) < 3:
            return
        mid = int(round(msg.data[0]))
        dist = float(msg.data[1])
        bearing = float(msg.data[2])
        # Donde el robot LEE el marcador, en el mundo (misma convencion que el EKF)
        ox = self.x + dist * np.cos(self.theta + bearing)
        oy = self.y + dist * np.sin(self.theta + bearing)
        self.last_obs = (mid, ox, oy)
        self.last_obs_count = 15        # ~1.5 s visible

        # Log de depuracion compacto: solo cuando el error supera el umbral.
        if mid in self.markers_map:
            mx, my = self.markers_map[mid]
            err = float(np.hypot(ox - mx, oy - my))
            if err > self.err_log_threshold:
                self.get_logger().warn(
                    f'aruco {mid} | error {err:.2f} m '
                    f'(leo en {ox:+.2f},{oy:+.2f} | mapa {mx:+.2f},{my:+.2f})',
                    throttle_duration_sec=1.0)
        else:
            self.get_logger().warn(
                f'aruco {mid} no esta en el mapa (revisa marker_ids).',
                throttle_duration_sec=2.0)

    # ---- Publicacion de markers ------------------------------------------
    def publish_markers(self):
        arr = MarkerArray()
        mid_counter = 0

        def new_marker(ns, mtype):
            nonlocal mid_counter
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = ns
            m.id = mid_counter
            mid_counter += 1
            m.type = mtype
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        # --- Mapa de marcadores: cubo azul + texto con el ID ---
        for mid, (mx, my) in self.markers_map.items():
            cube = new_marker('map_markers', Marker.CUBE)
            cube.pose.position.x = mx
            cube.pose.position.y = my
            cube.pose.position.z = 0.1
            cube.scale.x = 0.06
            cube.scale.y = 0.18
            cube.scale.z = 0.18
            cube.color = ColorRGBA(r=0.0, g=0.2, b=1.0, a=0.9)
            arr.markers.append(cube)

            txt = new_marker('map_ids', Marker.TEXT_VIEW_FACING)
            txt.pose.position.x = mx
            txt.pose.position.y = my
            txt.pose.position.z = 0.3
            txt.scale.z = 0.12
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.text = str(mid)
            arr.markers.append(txt)

        # --- Pose estimada del robot: flecha verde ---
        pose = new_marker('robot_pose', Marker.ARROW)
        pose.scale.x = 0.25
        pose.scale.y = 0.05
        pose.scale.z = 0.05
        pose.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.05
        pose.pose.orientation.z = float(np.sin(self.theta / 2.0))
        pose.pose.orientation.w = float(np.cos(self.theta / 2.0))
        arr.markers.append(pose)

        # --- Elipse de confianza (covarianza del EKF, ~2 sigma) ---
        # Crece con el dead-reckoning y se encoge al corregir con ArUco.
        Pxx, Pyy, Pxy = self.cov
        P = np.array([[Pxx, Pxy], [Pxy, Pyy]])
        vals, vecs = np.linalg.eigh(P)            # ascendente; vals[1] = eje mayor
        vals = np.clip(vals, 0.0, None)
        ang = float(np.arctan2(vecs[1, 1], vecs[0, 1]))
        k = 2.0                                   # ~95% de confianza
        # Diametros 2-sigma, acotados: minimo 0.15 m (siempre visible aunque la
        # covarianza sea ~0 al estar bien localizado) y maximo 1.5 m (que no llene
        # la pantalla al derivar). Se sigue notando crecer/encoger entre esos limites.
        major = float(np.clip(2.0 * k * np.sqrt(vals[1]), 0.20, 2.0))
        minor = float(np.clip(2.0 * k * np.sqrt(vals[0]), 0.20, 2.0))
        ell = new_marker('cov_ellipse', Marker.CYLINDER)
        ell.pose.position.x = self.x
        ell.pose.position.y = self.y
        ell.pose.position.z = 0.03
        ell.pose.orientation.z = float(np.sin(ang / 2.0))
        ell.pose.orientation.w = float(np.cos(ang / 2.0))
        ell.scale.x = major
        ell.scale.y = minor
        ell.scale.z = 0.03
        ell.color = ColorRGBA(r=1.0, g=0.85, b=0.0, a=0.55)
        arr.markers.append(ell)

        # --- Trayectoria: linea ---
        if len(self.path) > 1:
            path = new_marker('path', Marker.LINE_STRIP)
            path.scale.x = 0.02
            path.color = ColorRGBA(r=0.2, g=0.8, b=1.0, a=0.8)
            path.points = [Point(x=px, y=py, z=0.02) for px, py in self.path]
            arr.markers.append(path)

        # --- Lectura de ArUco: linea robot -> donde lo lee + esfera ---
        if self.last_obs is not None and self.last_obs_count > 0:
            self.last_obs_count -= 1
            mid, ox, oy = self.last_obs
            ray = new_marker('aruco_obs', Marker.LINE_LIST)
            ray.scale.x = 0.02
            ray.color = ColorRGBA(r=1.0, g=0.3, b=0.0, a=1.0)
            ray.points = [Point(x=self.x, y=self.y, z=0.1),
                          Point(x=ox, y=oy, z=0.1)]
            arr.markers.append(ray)

            sph = new_marker('aruco_obs', Marker.SPHERE)
            sph.pose.position.x = ox
            sph.pose.position.y = oy
            sph.pose.position.z = 0.1
            sph.scale.x = sph.scale.y = sph.scale.z = 0.1
            sph.color = ColorRGBA(r=1.0, g=0.3, b=0.0, a=1.0)
            arr.markers.append(sph)

        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = VizDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()