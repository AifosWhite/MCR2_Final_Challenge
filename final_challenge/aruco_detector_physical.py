#!/usr/bin/env python3
"""
aruco_detector_physical.py  —  Puente ArUco -> EKF (sin procesamiento de imagen).

En lugar de correr deteccion OpenCV (duplicando el trabajo de aruco_ros que ya
corre en aruco_jetson.launch.py), este nodo simplemente convierte la salida de
aruco_ros al formato Float32MultiArray que espera el EKF.

aruco_ros publica:
  /aruco_ros/markers  (aruco_msgs/msg/MarkerArray)
    marker.id
    marker.pose.pose.position.{x, y, z}   <- posicion en frame camara
      x: lateral (+ derecha)
      y: vertical (+ abajo)
      z: profundidad (distancia frontal)

Este nodo publica:
  /aruco/detections  (std_msgs/Float32MultiArray)
    [marker_id, distance, bearing]
    distance = sqrt(x^2 + z^2)   <- distancia en el plano del piso
    bearing  = -atan2(x, z)      <- angulo respecto al frente, + izquierda

Solo publica el marcador mas cercano por ciclo (igual que antes).
"""

import math
import signal
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

try:
    from aruco_msgs.msg import MarkerArray
    _ARUCO_MSGS = True
except ImportError:
    _ARUCO_MSGS = False


class ArucoDetectorPhysical(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        if not _ARUCO_MSGS:
            self.get_logger().error(
                'aruco_msgs no disponible. Instala el paquete aruco_ros.')
            raise RuntimeError('aruco_msgs no disponible')

        self.declare_parameter('aruco_ros_topic', '/aruco_ros/markers')
        self.declare_parameter('marker_ids',
                               [70, 75, 701, 702, 703, 705, 706, 708])

        aruco_topic = str(self.get_parameter('aruco_ros_topic').value)
        ids = list(self.get_parameter('marker_ids').value)
        self.valid_ids = set(int(i) for i in ids)

        self.create_subscription(MarkerArray, aruco_topic,
                                 self.markers_callback, 10)
        self.publisher = self.create_publisher(
            Float32MultiArray, '/aruco/detections', 10)

        signal.signal(signal.SIGINT, self.shutdown_function)
        self.get_logger().info(
            f'aruco_detector (puente) listo. '
            f'Leyendo {aruco_topic} → /aruco/detections')

    def markers_callback(self, msg: 'MarkerArray'):
        if not msg.markers:
            return

        closest = None
        closest_dist = float('inf')

        for marker in msg.markers:
            marker_id = int(marker.id)
            if marker_id not in self.valid_ids:
                continue

            # Posicion en frame optico de la camara:
            # x = lateral (+ derecha), y = vertical (+ abajo), z = profundidad
            tx = float(marker.pose.pose.position.x)
            tz = float(marker.pose.pose.position.z)

            distance = math.sqrt(tx * tx + tz * tz)
            bearing  = -math.atan2(tx, tz)   # + izquierda (convencion EKF)

            if distance < closest_dist:
                closest_dist = distance
                closest = (float(marker_id), distance, bearing)

        if closest is not None:
            arr = Float32MultiArray()
            arr.data = list(closest)
            self.publisher.publish(arr)
            self.get_logger().info(
                f'ArUco {int(closest[0])} | '
                f'dist={closest[1]:.3f} m  bearing={math.degrees(closest[2]):+.1f} deg',
                throttle_duration_sec=0.5)

    def shutdown_function(self, signum, frame):
        self.get_logger().info('Shutting down aruco_detector...')
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorPhysical()
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