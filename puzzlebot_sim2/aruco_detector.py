import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
import cv2
import math

def yaw_to_quat(yaw):
    # Returns (x, y, z, w) for yaw-only quaternion
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Parameters
        self.declare_parameter('marker_size_m', 0.18)
        self.declare_parameter('fx', 554.0)
        self.declare_parameter('fy', 554.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('k1', 0.0)
        self.declare_parameter('k2', 0.0)
        self.declare_parameter('p1', 0.0)
        self.declare_parameter('p2', 0.0)
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_1000')

        self.marker_size = float(self.get_parameter('marker_size_m').value)
        self.fx = float(self.get_parameter('fx').value)
        self.fy = float(self.get_parameter('fy').value)
        self.cx = float(self.get_parameter('cx').value)
        self.cy = float(self.get_parameter('cy').value)
        self.k1 = float(self.get_parameter('k1').value)
        self.k2 = float(self.get_parameter('k2').value)
        self.p1 = float(self.get_parameter('p1').value)
        self.p2 = float(self.get_parameter('p2').value)
        self.aruco_dictionary = str(self.get_parameter('aruco_dictionary').value)

        self.camera_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ], dtype=np.float32)
        self.dist_coeffs = np.array([self.k1, self.k2, self.p1, self.p2, 0.0], dtype=np.float32)

        dictionary_id = getattr(cv2.aruco, self.aruco_dictionary, cv2.aruco.DICT_4X4_1000)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.aruco_params = cv2.aruco.DetectorParameters()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.aruco_detector = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, qos)
        self.pose_pubs = {}
        self.detections_pub = self.create_publisher(Float32MultiArray, '/aruco/detections', 10)

        self.new_image = False
        self.last_image = None
        self.last_header = None

        self.timer = self.create_timer(0.05, self.timer_callback)

    def image_callback(self, msg):
        self.last_image = msg
        self.last_header = msg.header
        self.new_image = True

    def timer_callback(self):
        if not self.new_image or self.last_image is None:
            return
        msg = self.last_image
        self.new_image = False

        # Convert ROS Image to cv2 BGR
        if msg.encoding == 'rgb8':
            img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape((msg.height, msg.width, 3))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'bgr8':
            img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape((msg.height, msg.width, 3))
        elif msg.encoding in ('mono8', '8UC1'):
            img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape((msg.height, msg.width))
        else:
            self.get_logger().warn('Unsupported image encoding: %s' % msg.encoding)
            return

        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(img)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_params)
        if ids is None or len(ids) == 0:
            return

        ids = ids.flatten()
        self.get_logger().debug(f'Detected marker IDs: {ids.tolist()}')

        marker_poses = []
        for i, marker_id in enumerate(ids):
            c = corners[i][0]
            objp = np.array([
                [-self.marker_size/2,  self.marker_size/2, 0],
                [ self.marker_size/2,  self.marker_size/2, 0],
                [ self.marker_size/2, -self.marker_size/2, 0],
                [-self.marker_size/2, -self.marker_size/2, 0]
            ], dtype=np.float32)
            imgp = c.astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(objp, imgp, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not success:
                continue
            R, _ = cv2.Rodrigues(rvec)
            yaw = math.atan2(R[1][0], R[0][0])
            distance = math.sqrt(tvec[0][0]**2 + tvec[1][0]**2 + tvec[2][0]**2)
            bearing = math.atan2(tvec[0][0], tvec[2][0])

            # Publish PoseStamped
            topic = f'/aruco/marker_{marker_id}'
            if marker_id not in self.pose_pubs:
                self.pose_pubs[marker_id] = self.create_publisher(PoseStamped, topic, 10)
            pose = PoseStamped()
            pose.header.stamp = self.last_header.stamp
            pose.header.frame_id = 'base_link'
            pose.pose.position.x = float(tvec[0][0])
            pose.pose.position.y = float(tvec[1][0])
            pose.pose.position.z = float(tvec[2][0])
            qx, qy, qz, qw = yaw_to_quat(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            self.pose_pubs[marker_id].publish(pose)

            marker_poses.append((marker_id, distance, bearing))

        if marker_poses:
            # Publish closest marker
            marker_poses.sort(key=lambda x: x[1])
            marker_id, distance, bearing = marker_poses[0]
            arr = Float32MultiArray()
            arr.data = [float(marker_id), float(distance), float(bearing)]
            self.detections_pub.publish(arr)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
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
