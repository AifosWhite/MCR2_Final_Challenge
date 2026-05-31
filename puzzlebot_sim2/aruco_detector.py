import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import numpy as np
import math
import cv2


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.marker_size = 0.14
        fx = 554.0
        fy = 554.0
        cx = 320.0
        cy = 240.0
        k1 = 0.0
        k2 = 0.0
        p1 = 0.0
        p2 = 0.0
        dict_name = 'DICT_4X4_1000'
        self.camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([k1, k2, p1, p2, 0.0], dtype=np.float32)
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name, cv2.aruco.DICT_4X4_1000))
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, cv2.aruco.DetectorParameters())
        self.publisher = self.create_publisher(Float32MultiArray, '/aruco/detections', 10)
        qos = rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, qos)
        self.new_image = False
        self.latest_msg = None
        self.create_timer(0.05, self.timer_callback)

    def image_callback(self, msg):
        self.latest_msg = msg
        self.new_image = True

    def timer_callback(self):
        if not self.new_image or self.latest_msg is None:
            return
        self.new_image = False
        msg = self.latest_msg
        if msg.encoding == 'rgb8':
            img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape((msg.height, msg.width, 3))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'bgr8':
            img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape((msg.height, msg.width, 3))
        else:
            return
        corners, ids, _ = self.detector.detectMarkers(img)
        if ids is None or len(ids) == 0:
            return
        markers = []
        for i, marker_id in enumerate(ids.flatten()):
            objp = np.array([
                [-self.marker_size/2,  self.marker_size/2, 0],
                [ self.marker_size/2,  self.marker_size/2, 0],
                [ self.marker_size/2, -self.marker_size/2, 0],
                [-self.marker_size/2, -self.marker_size/2, 0]
            ], dtype=np.float32)
            imgp = corners[i][0].astype(np.float32)
            ok, rvec, tvec = cv2.solvePnP(objp, imgp, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            tx, ty, tz = tvec[0][0], tvec[1][0], tvec[2][0]
            distance = math.sqrt(tx**2 + ty**2 + tz**2)
            bearing = math.atan2(tx, tz)
            markers.append((float(marker_id), distance, bearing))
        if markers:
            closest = min(markers, key=lambda m: m[1])
            arr = Float32MultiArray()
            arr.data = [closest[0], closest[1], closest[2]]
            self.publisher.publish(arr)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
