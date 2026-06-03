#!/usr/bin/env python3
"""
Local ArUco detector that reads from /video_source/raw and detects markers locally.
Useful for debugging: shows which markers are visible and their distance.
"""
import cv2
import cv2.aruco as aruco
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import sys
from std_msgs.msg import Float32MultiArray
import rclpy.logging
from rclpy.qos import QoSProfile, ReliabilityPolicy

class LocalArucoDetector(Node):
    def __init__(self):
        super().__init__('local_aruco_detector')
        # increase logger verbosity for debugging
        try:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        except Exception:
            pass
        self.bridge = CvBridge()
        # Subscribe to several candidate image topics used in this workspace
        topics = [
            '/video_source/raw',
            '/video_source',
            '/marker_publisher/result',
            '/marker_publisher/result/compressed',
            '/video_source/raw/compressed',
            '/video_source/compressed',
            '/camera/image_raw',
            '/camera/image_raw/compressed',
        ]
        self.subs = []
        for t in topics:
            try:
                if t.endswith('/compressed'):
                    sub = self.create_subscription(CompressedImage, t, lambda msg, tn=t: self.cb_compressed(msg, tn), 10)
                else:
                    sub = self.create_subscription(Image, t, lambda msg, tn=t: self.cb(msg, tn), 10)
                self.subs.append((t, sub))
                self.get_logger().info(f'Subscribed to {t}')
            except Exception:
                pass
        self.target_id = None
        
        # ArUco dictionary and detector parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        # Create detector parameters compatible with multiple OpenCV versions
        try:
            self.parameters = aruco.DetectorParameters_create()
        except AttributeError:
            # Older OpenCV may expose DetectorParameters_create under different name
            try:
                self.parameters = aruco.DetectorParameters()
            except Exception:
                self.parameters = None

        # Prefer ArucoDetector if available (new API), otherwise fallback to detectMarkers
        self.have_aruco_detector = hasattr(aruco, 'ArucoDetector') and self.parameters is not None
        if self.have_aruco_detector:
            self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        else:
            self.detector = None
        
        # Camera parameters (from calibration, or defaults)
        self.camera_matrix = np.array([
            [500, 0, 160],
            [0, 500, 120],
            [0, 0, 1]
        ], dtype='float32')
        self.dist_coeffs = np.zeros((4, 1))
        self.marker_size = 0.094  # meters
        # Publisher for detected markers: [id, distance_m, bearing_rad]
        qos_pub = QoSProfile(depth=10)
        qos_pub.reliability = ReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(Float32MultiArray, '/aruco/detections', qos_pub)

        # Heartbeat timer: publish a sentinel when no detection to keep topic visible
        self.heartbeat_timer = self.create_timer(1.0, self._publish_heartbeat)
        self._last_detection_time = 0.0

    def cb(self, msg: Image, topic_name: str = '/video_source/raw'):
        self.get_logger().debug(f'Received Image on {topic_name}')
        try:
            # Try several common encodings
            for enc in ('bgr8', 'rgb8', 'mono8'):
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=enc)
                    break
                except Exception:
                    cv_image = None
            if cv_image is None:
                # fallback: try without specifying encoding
                cv_image = self.bridge.imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().warn(f"Bridge error: {e}")
            return
        self.get_logger().debug(f'Received image msg (size={msg.width}x{msg.height}) on {topic_name}')
        self._process_image(cv_image, topic_name)

    def cb_compressed(self, msg: CompressedImage, topic_name: str = '/video_source/raw/compressed'):
        self.get_logger().debug(f'Received CompressedImage on {topic_name}')
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                self.get_logger().warn(f'Failed to decode compressed image from {topic_name}')
                return
            self._process_image(cv_image, topic_name)
        except Exception as e:
            self.get_logger().warn(f'Compressed image handling failed: {e}')

    def _process_image(self, cv_image: np.ndarray, topic_name: str):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        if self.have_aruco_detector and self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            # fallback to older API
            try:
                corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            except Exception:
                corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict)

        if ids is not None:
            self.get_logger().info(f"Detected {len(ids)} markers on {topic_name}")
            for i, marker_id in enumerate(ids.flatten()):
                corner = corners[i][0]
                marker_size_pixels = np.linalg.norm(corner[0] - corner[2])
                focal_length = 500
                distance = (self.marker_size * focal_length) / marker_size_pixels if marker_size_pixels > 0 else 0
                cv2.polylines(cv_image, [np.int32(corner)], True, (0, 255, 0), 2)
                center = np.mean(corner, axis=0).astype(int)
                text = f"ID={marker_id} d={distance:.2f}m"
                cv2.putText(cv_image, text, tuple(center), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                if self.target_id is None or marker_id == self.target_id:
                    self.get_logger().info(f"ID={marker_id} distance={distance:.3f}m")
                cx = self.camera_matrix[0, 2]
                focal = self.camera_matrix[0, 0]
                bearing = float(np.arctan2((center[0] - cx), focal))
                msg = Float32MultiArray()
                msg.data = [float(marker_id), float(distance), bearing]
                try:
                    self.pub.publish(msg)
                    self.get_logger().debug(f'Published /aruco/detections: {msg.data}')
                    self._last_detection_time = self.get_clock().now().seconds_nanoseconds()[0]
                except Exception as e:
                    self.get_logger().warn(f'Failed to publish detection: {e}')
        else:
            self.get_logger().debug(f'No markers detected on {topic_name}')
        cv2.imshow('ArUco Detection', cv_image)
        cv2.waitKey(1)

    def _publish_heartbeat(self):
        """Publish a sentinel detection periodically so topic is visible to other nodes.

        If a real detection was published recently, skip to avoid noise.
        """
        try:
            now = self.get_clock().now().seconds_nanoseconds()[0]
            # publish heartbeat if no detection in last 1.5 seconds
            if now - self._last_detection_time > 1.5:
                msg = Float32MultiArray()
                msg.data = [-1.0, 0.0, 0.0]
                self.pub.publish(msg)
                self.get_logger().debug('Published heartbeat on /aruco/detections')
        except Exception:
            # ignore errors during shutdown
            pass


def main():
    if len(sys.argv) > 1:
        try:
            target_id = int(sys.argv[1])
            print(f"Filtering for marker ID {target_id}")
        except:
            target_id = None
    else:
        target_id = None
    
    rclpy.init()
    node = LocalArucoDetector()
    node.target_id = target_id
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        # rclpy may already be shutting down (e.g. KeyboardInterrupt); ignore
        pass

    # end of main

    # end of file

if __name__ == '__main__':
    main()
