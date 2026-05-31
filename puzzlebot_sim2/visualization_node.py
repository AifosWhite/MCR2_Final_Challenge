import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


def yaw_to_quat(yaw):
    return math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)


class LocalisationVisualization(Node):
    def __init__(self):
        super().__init__('localisation_visualization')

        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('fov_deg', 60.0)
        self.declare_parameter('vision_range', 2.5)
        self.declare_parameter('covariance_sigma', 3.0)
        self.declare_parameter('covariance_scale', 12.0)
        self.declare_parameter('min_covariance_axis', 0.05)
        self.declare_parameter('path_keep', 500)

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.fov = math.radians(float(self.get_parameter('fov_deg').value))
        self.vision_range = float(self.get_parameter('vision_range').value)
        self.covariance_sigma = float(self.get_parameter('covariance_sigma').value)
        self.covariance_scale = float(self.get_parameter('covariance_scale').value)
        self.min_covariance_axis = float(self.get_parameter('min_covariance_axis').value)
        self.path_keep = int(self.get_parameter('path_keep').value)

        self.path = []
        self.marker_pub = self.create_publisher(MarkerArray, 'localisation_markers', 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.get_logger().info('Visualizacion lista: covarianza, campo de vision y trayectoria.')

    def odom_callback(self, msg):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        self.path.append((x, y))
        if len(self.path) > self.path_keep:
            self.path = self.path[-self.path_keep:]

        markers = MarkerArray()
        markers.markers.append(self.covariance_marker(msg))
        markers.markers.append(self.vision_marker(msg.header.stamp))
        markers.markers.append(self.vision_outline_marker(msg.header.stamp))
        markers.markers.append(self.path_marker(msg))
        self.marker_pub.publish(markers)

    def covariance_marker(self, msg):
        cov = np.array([
            [msg.pose.covariance[0], msg.pose.covariance[1]],
            [msg.pose.covariance[6], msg.pose.covariance[7]],
        ], dtype=float)
        cov = 0.5 * (cov + cov.T)

        values, vectors = np.linalg.eigh(cov)
        values = np.maximum(values, 0.0)
        order = np.argsort(values)[::-1]
        values = values[order]
        vectors = vectors[:, order]

        major = max(
            2.0 * self.covariance_sigma * math.sqrt(values[0]) * self.covariance_scale,
            self.min_covariance_axis,
        )
        minor = max(
            2.0 * self.covariance_sigma * math.sqrt(values[1]) * self.covariance_scale,
            self.min_covariance_axis,
        )
        yaw = math.atan2(vectors[1, 0], vectors[0, 0])
        q = yaw_to_quat(yaw)

        marker = Marker()
        marker.header = msg.header
        marker.ns = 'localisation_covariance'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = float(msg.pose.pose.position.x)
        marker.pose.position.y = float(msg.pose.pose.position.y)
        marker.pose.position.z = 0.03
        marker.pose.orientation.w = q[0]
        marker.pose.orientation.x = q[1]
        marker.pose.orientation.y = q[2]
        marker.pose.orientation.z = q[3]
        marker.scale.x = float(major)
        marker.scale.y = float(minor)
        marker.scale.z = 0.02
        marker.color.r = 0.55
        marker.color.g = 0.62
        marker.color.b = 0.68
        marker.color.a = 0.45
        return marker

    def vision_marker(self, stamp):
        half = self.fov / 2.0
        points = [
            (0.0, 0.0),
            (self.vision_range * math.cos(half), self.vision_range * math.sin(half)),
            (self.vision_range * math.cos(-half), self.vision_range * math.sin(-half)),
        ]

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.base_frame
        marker.ns = 'aruco_vision'
        marker.id = 1
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.05
        marker.color.a = 0.35
        for px, py in points:
            point = Point()
            point.x = float(px)
            point.y = float(py)
            point.z = 0.04
            marker.points.append(point)
        return marker

    def vision_outline_marker(self, stamp):
        half = self.fov / 2.0
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.base_frame
        marker.ns = 'aruco_vision'
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.025
        marker.color.r = 0.85
        marker.color.g = 0.65
        marker.color.b = 0.0
        marker.color.a = 0.9
        outline = [
            (0.0, 0.0),
            (self.vision_range * math.cos(half), self.vision_range * math.sin(half)),
            (self.vision_range * math.cos(-half), self.vision_range * math.sin(-half)),
            (0.0, 0.0),
        ]
        for px, py in outline:
            point = Point()
            point.x = float(px)
            point.y = float(py)
            point.z = 0.05
            marker.points.append(point)
        return marker

    def path_marker(self, msg):
        marker = Marker()
        marker.header = msg.header
        marker.ns = 'localisation_path'
        marker.id = 3
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.025
        marker.color.r = 1.0
        marker.color.g = 0.05
        marker.color.b = 0.02
        marker.color.a = 0.95
        for x, y in self.path:
            point = Point()
            point.x = float(x)
            point.y = float(y)
            point.z = 0.06
            marker.points.append(point)
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = LocalisationVisualization()
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
