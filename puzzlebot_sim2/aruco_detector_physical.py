#!/usr/bin/env python3
"""
aruco_detector_physical.py  -  Detector de ArUco para el Puzzlebot FISICO.

Ruta A: detector propio que publica (id, distance, bearing) en /aruco/detections,
exactamente el formato que tu localisation.py (EKF) ya consume. No requiere
aruco_opencv ni mensajes custom.

Diferencias clave vs. tu aruco_detector.py de simulacion:
  - Matriz de camara y distorsion son PARAMETROS (no hardcode de sim).
    DEBES poner los valores reales de calibracion de TU camara.
  - Topico de imagen por defecto: /video_source/raw (el del Puzzlebot fisico).
  - La distancia/bearing se calculan en el plano del robot. Para mayor
    precision puedes habilitar el uso de TF (use_tf:=true) para transformar
    al frame base_footprint, como hace el equipo de referencia.
  - El diccionario es parametro: en fisico tus marcadores son DICT_4X4_1000.

Publica SOLO el marcador mas cercano por ciclo (igual que tu version de sim),
porque tu EKF corrige con una sola observacion por paso.
"""

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

try:
    from geometry_msgs.msg import PointStamped
    from tf2_ros import Buffer, TransformListener
    import tf2_geometry_msgs  # noqa: F401  (registra el tipo para transform)
    _TF_AVAILABLE = True
except Exception:
    _TF_AVAILABLE = False


class ArucoDetectorPhysical(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # ---- Parametros de camara (PON LOS DE TU CALIBRACION REAL) --------
        self.declare_parameter('marker_size_m', 0.14)          # marcador impreso
        self.declare_parameter('image_topic', '/video_source/raw')
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_1000')
        self.declare_parameter('use_tf', False)
        self.declare_parameter('camera_optical_frame', 'camera_link_optical')
        self.declare_parameter('base_frame', 'base_footprint')
        # fx, fy, cx, cy  -> matriz intrinseca. Estos son PLACEHOLDERS.
        self.declare_parameter('camera_matrix',
                               [191.26581, 0.0, 169.60164,
                                0.0, 255.02285, 109.55441,
                                0.0, 0.0, 1.0])
        self.declare_parameter('camera_distortion',
                               [-0.348494, 0.113557, -0.000324, 0.000785, 0.0])

        self.marker_size = float(self.get_parameter('marker_size_m').value)
        image_topic = str(self.get_parameter('image_topic').value)
        dict_name = str(self.get_parameter('aruco_dictionary').value)
        self.use_tf = bool(self.get_parameter('use_tf').value) and _TF_AVAILABLE
        self.camera_optical_frame = str(
            self.get_parameter('camera_optical_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        cm = list(self.get_parameter('camera_matrix').value)
        self.camera_matrix = np.array(cm, dtype=np.float32).reshape((3, 3))
        dc = list(self.get_parameter('camera_distortion').value)
        self.dist_coeffs = np.array(dc, dtype=np.float32).reshape((-1, 1))

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dict_name, cv2.aruco.DICT_4X4_1000))
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(
                self.dictionary, cv2.aruco.DetectorParameters())
            self.detector_parameters = None
        else:
            self.detector = None
            if hasattr(cv2.aruco, 'DetectorParameters_create'):
                self.detector_parameters = cv2.aruco.DetectorParameters_create()
            else:
                self.detector_parameters = cv2.aruco.DetectorParameters()

        # ---- TF opcional --------------------------------------------------
        if self.use_tf:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- ROS ----------------------------------------------------------
        self.publisher = self.create_publisher(
            Float32MultiArray, '/aruco/detections', 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, qos)

        self.new_image = False
        self.latest_msg = None
        self._logged_encoding = False
        self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            f'aruco_detector (fisico) listo. Imagen={image_topic}, '
            f'dict={dict_name}, marker={self.marker_size} m, '
            f'use_tf={self.use_tf}.')
        self.get_logger().warn(
            'RECORDATORIO: pon la matriz de camara y distorsion REALES de tu '
            'Puzzlebot via parametros. Los valores por defecto NO son los tuyos.')

    def image_callback(self, msg):
        self.latest_msg = msg
        self.new_image = True

    def _decode(self, msg):
        if msg.encoding == 'rgb8':
            img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
                (msg.height, msg.width, 3))
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if msg.encoding == 'bgr8':
            return np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
                (msg.height, msg.width, 3))
        if msg.encoding in ('mono8', '8UC1'):
            gray = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(
                (msg.height, msg.width))
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        self.get_logger().warn(
            f'Encoding no soportado: "{msg.encoding}".',
            throttle_duration_sec=5.0)
        return None

    def timer_callback(self):
        if not self.new_image or self.latest_msg is None:
            return
        self.new_image = False
        msg = self.latest_msg

        if not self._logged_encoding:
            self._logged_encoding = True
            self.get_logger().info(
                f'Primera imagen: encoding="{msg.encoding}", '
                f'{msg.width}x{msg.height}.')

        img = self._decode(msg)
        if img is None:
            return

        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(img)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                img, self.dictionary, parameters=self.detector_parameters)

        if ids is None or len(ids) == 0:
            return
        self.get_logger().info(
            f'Detectados IDs: {ids.flatten().tolist()}.',
            throttle_duration_sec=1.0)

        markers = []
        for i, marker_id in enumerate(ids.flatten()):
            objp = np.array([
                [-self.marker_size / 2,  self.marker_size / 2, 0],
                [ self.marker_size / 2,  self.marker_size / 2, 0],
                [ self.marker_size / 2, -self.marker_size / 2, 0],
                [-self.marker_size / 2, -self.marker_size / 2, 0],
            ], dtype=np.float32)
            imgp = corners[i][0].astype(np.float32)
            ok, rvec, tvec = cv2.solvePnP(
                objp, imgp, self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue

            tx, ty, tz = float(tvec[0]), float(tvec[1]), float(tvec[2])

            if self.use_tf:
                obs = self._observe_via_tf(msg, tx, ty, tz)
                if obs is None:
                    continue
                distance, bearing = obs
            else:
                # Frame optico: x derecha, y abajo, z al frente.
                # distancia en el plano del piso, bearing respecto al frente (z).
                distance = math.sqrt(tx * tx + tz * tz)
                bearing = math.atan2(tx, tz)
                # bearing>0 a la derecha de la camara. Tu EKF usa atan2(dy,dx)-theta,
                # con bearing positivo a la izquierda; por eso invertimos el signo.
                bearing = -bearing

            markers.append((float(marker_id), distance, bearing))

        if markers:
            closest = min(markers, key=lambda m: m[1])
            arr = Float32MultiArray()
            arr.data = [closest[0], closest[1], closest[2]]
            self.publisher.publish(arr)

    def _observe_via_tf(self, msg, tx, ty, tz):
        try:
            pt = PointStamped()
            pt.header.stamp = msg.header.stamp
            pt.header.frame_id = self.camera_optical_frame.lstrip('/')
            pt.point.x = tx
            pt.point.y = ty
            pt.point.z = tz
            tpt = self.tf_buffer.transform(
                pt, self.base_frame,
                timeout=rclpy.duration.Duration(seconds=0.2))
            dx, dy = tpt.point.x, tpt.point.y
            distance = math.hypot(dx, dy)
            bearing = math.atan2(dy, dx)
            return distance, bearing
        except Exception as exc:  # TF no disponible aun
            self.get_logger().warn(
                f'TF no disponible ({exc}); usa use_tf:=false si persiste.',
                throttle_duration_sec=5.0)
            return None


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
