#!/usr/bin/env python3

import math
import io
import os
from typing import List

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('marker_length', 0.18)  # meters
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('camera_fov', 1.047)  # horizontal FOV in radians
        self.declare_parameter('dictionary', 'DICT_4X4_250')

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.marker_length = float(self.get_parameter('marker_length').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.parent_frame = str(self.get_parameter('parent_frame').value)
        self.camera_fov = float(self.get_parameter('camera_fov').value)
        dict_name = str(self.get_parameter('dictionary').value)

        # prepare aruco dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict)

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
        )

        # camera pose relative to base_link - should match URDF pose used in the Gazebo <gazebo> sensor block
        # If you change URDF, update these values via ROS parameters
        self.camera_position_wrt_base = np.array([0.10, 0.0, 0.10])
        self.camera_rpy_wrt_base = np.array([0.0, 0.0, 0.0])

        self.get_logger().info(f'Aruco detector listening to {self.image_topic}')

    def image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warning(f'Could not convert image: {e}')
            return

        h, w = cv_img.shape[:2]
        # build camera matrix from fov and image size
        fx = (w / 2.0) / math.tan(self.camera_fov / 2.0)
        fy = fx
        cx = w / 2.0
        cy = h / 2.0
        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=float)
        dist_coeffs = np.zeros((5, 1))

        # detect
        corners, ids, rejected = self.detector.detectMarkers(cv_img)
        if ids is None or len(ids) == 0:
            return

        # estimate pose for each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, camera_matrix, dist_coeffs)

        # compute base_R_camera from rpy
        cr = math.cos(self.camera_rpy_wrt_base[0]); sr = math.sin(self.camera_rpy_wrt_base[0])
        cp = math.cos(self.camera_rpy_wrt_base[1]); sp = math.sin(self.camera_rpy_wrt_base[1])
        cyaw = math.cos(self.camera_rpy_wrt_base[2]); syaw = math.sin(self.camera_rpy_wrt_base[2])

        R_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        R_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        R_z = np.array([[cyaw, -syaw, 0], [syaw, cyaw, 0], [0, 0, 1]])
        R_base_camera = R_z @ R_y @ R_x

        for i, marker_id in enumerate(ids.flatten()):
            t_cam_marker = tvecs[i].reshape((3,))
            r_cam_marker, _ = cv2.Rodrigues(rvecs[i])

            # base_T_marker = base_T_camera * camera_T_marker
            R_base_marker = R_base_camera @ r_cam_marker
            p_base_marker = self.camera_position_wrt_base + R_base_camera @ t_cam_marker

            # build quaternion from rotation matrix
            quat = self.rotation_matrix_to_quaternion(R_base_marker)

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.parent_frame
            t.child_frame_id = f'marker_{marker_id}'
            t.transform.translation.x = float(p_base_marker[0])
            t.transform.translation.y = float(p_base_marker[1])
            t.transform.translation.z = float(p_base_marker[2])
            t.transform.rotation.x = float(quat[0])
            t.transform.rotation.y = float(quat[1])
            t.transform.rotation.z = float(quat[2])
            t.transform.rotation.w = float(quat[3])

            self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
        # Converts a proper rotation matrix into a quaternion [x, y, z, w].
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
        return np.array([qx, qy, qz, qw], dtype=float)


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
