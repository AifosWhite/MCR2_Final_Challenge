import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import math
import cv2


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.declare_parameter('marker_size_m', 0.20)
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('dict_name', 'DICT_5X5_100')  # Changed to more robust dictionary
        
        self.marker_size = float(self.get_parameter('marker_size_m').value)
        self.image_topic = self.get_parameter('image_topic').value
        dict_name = self.get_parameter('dict_name').value
        
        # Camera intrinsics (hardcoded for now)
        fx = 554.0
        fy = 554.0
        cx = 320.0
        cy = 240.0
        k1 = 0.0
        k2 = 0.0
        p1 = 0.0
        p2 = 0.0
        
        self.camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([k1, k2, p1, p2, 0.0], dtype=np.float32)
        
        # ArUco detector setup
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name, cv2.aruco.DICT_4X4_1000))
        if hasattr(cv2.aruco, 'ArucoDetector'):
            # Configure detector parameters to be more restrictive
            detector_params = cv2.aruco.DetectorParameters()
            detector_params.adaptiveThreshConstant = 7
            detector_params.minMarkerPerimeterRate = 0.05
            detector_params.maxMarkerPerimeterRate = 4.0
            detector_params.polygonalApproxAccuracyRate = 0.05
            detector_params.minCornerDistanceRate = 0.05
            detector_params.minDistanceToBorder = 1
            detector_params.minMarkerDistanceRate = 0.05
            detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, detector_params)
            self.detector_parameters = None
        else:
            self.detector = None
            if hasattr(cv2.aruco, 'DetectorParameters_create'):
                self.detector_parameters = cv2.aruco.DetectorParameters_create()
            else:
                self.detector_parameters = cv2.aruco.DetectorParameters()
                # Try to set parameters for older OpenCV
                if hasattr(self.detector_parameters, 'adaptiveThreshConstant'):
                    self.detector_parameters.adaptiveThreshConstant = 7
                if hasattr(self.detector_parameters, 'minMarkerPerimeterRate'):
                    self.detector_parameters.minMarkerPerimeterRate = 0.05
                if hasattr(self.detector_parameters, 'polygonalApproxAccuracyRate'):
                    self.detector_parameters.polygonalApproxAccuracyRate = 0.05
        
        self.cv_bridge = CvBridge()
        self.detection_publisher = self.create_publisher(Float32MultiArray, '/aruco/detections', 10)
        
        # Image publisher with BEST_EFFORT QoS to match subscriber
        image_qos = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.image_publisher = self.create_publisher(Image, '/aruco/result', image_qos)
        
        qos = rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(Image, self.image_topic, self.image_callback, qos)
        
        self.new_image = False
        self.latest_msg = None
        self._logged_encoding = False
        self._detection_count = 0
        
        self.create_timer(0.05, self.timer_callback)
        self.get_logger().info(f'ArUco Detector initialized. Subscribing to: {self.image_topic}')

    def image_callback(self, msg):
        self.latest_msg = msg
        self.new_image = True

    def timer_callback(self):
        if not self.new_image or self.latest_msg is None:
            return
        self.new_image = False
        msg = self.latest_msg
        
        if not self._logged_encoding:
            self._logged_encoding = True
            self.get_logger().info(
                f'Imagen recibida: encoding="{msg.encoding}", {msg.width}x{msg.height}.')
        
        try:
            # Convert image
            if msg.encoding == 'rgb8':
                img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape((msg.height, msg.width, 3))
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape((msg.height, msg.width, 3))
            else:
                self.get_logger().warn(
                    f'Encoding no soportado: "{msg.encoding}" (esperado rgb8/bgr8).',
                    throttle_duration_sec=5.0)
                return
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}', throttle_duration_sec=5.0)
            return
        
        # Make a copy for annotation
        annotated_img = img.copy()
        
        try:
            # Detect markers
            if self.detector is not None:
                corners, ids, rejected = self.detector.detectMarkers(img)
            else:
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    img,
                    self.dictionary,
                    parameters=self.detector_parameters,
                )
        except Exception as e:
            self.get_logger().error(f'Error detectando marcadores: {e}', throttle_duration_sec=5.0)
            # Still publish empty result
            try:
                result_msg = self.cv_bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8")
                result_msg.header = msg.header
                self.image_publisher.publish(result_msg)
            except:
                pass
            return
        
        # Draw detected markers
        if ids is not None and len(ids) > 0:
            try:
                if hasattr(cv2.aruco, 'drawDetectedMarkers'):
                    annotated_img = cv2.aruco.drawDetectedMarkers(annotated_img, corners, ids)
                else:
                    # Fallback for older OpenCV
                    for i, corner in enumerate(corners):
                        corner_int = corner[0].astype(int)
                        cv2.polylines(annotated_img, [corner_int], True, (0, 255, 0), 2)
                        if ids is not None and i < len(ids):
                            cv2.putText(annotated_img, f"{int(ids[i][0])}", 
                                      tuple(corner_int[0]), cv2.FONT_HERSHEY_SIMPLEX, 
                                      0.5, (0, 0, 255), 2)
            except Exception as e:
                self.get_logger().error(f'Error dibujando marcadores: {e}', throttle_duration_sec=5.0)
            
            self._detection_count += 1
            self.get_logger().info(
                f'[#{self._detection_count}] Detectados {len(ids)} IDs: {ids.flatten().tolist()}',
                throttle_duration_sec=1.0)
            
            # Compute detection pose data
            try:
                markers = []
                for i, marker_id in enumerate(ids.flatten()):
                    if i >= len(corners):
                        break
                    
                    objp = np.array([
                        [-self.marker_size/2,  self.marker_size/2, 0],
                        [ self.marker_size/2,  self.marker_size/2, 0],
                        [ self.marker_size/2, -self.marker_size/2, 0],
                        [-self.marker_size/2, -self.marker_size/2, 0]
                    ], dtype=np.float32)
                    imgp = corners[i][0].astype(np.float32)
                    
                    ok, rvec, tvec = cv2.solvePnP(objp, imgp, self.camera_matrix, 
                                                  self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if not ok:
                        self.get_logger().warn(f'solvePnP failed for marker {marker_id}')
                        continue
                    
                    tx, ty, tz = tvec[0][0], tvec[1][0], tvec[2][0]
                    distance = math.sqrt(tx**2 + ty**2 + tz**2)
                    bearing = math.atan2(tx, tz)
                    markers.append((float(marker_id), distance, bearing))
                
                if markers:
                    closest = min(markers, key=lambda m: m[1])
                    arr = Float32MultiArray()
                    arr.data = [closest[0], closest[1], closest[2]]
                    self.detection_publisher.publish(arr)
                    self.get_logger().debug(
                        f'Publishado: marker_id={int(closest[0])}, distance={closest[1]:.3f}m, bearing={closest[2]:.3f}rad')
            except Exception as e:
                self.get_logger().error(f'Error computando pose: {e}', throttle_duration_sec=5.0)
        
        # Always publish annotated result image
        try:
            result_msg = self.cv_bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8")
            result_msg.header = msg.header
            self.image_publisher.publish(result_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishando imagen: {e}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

