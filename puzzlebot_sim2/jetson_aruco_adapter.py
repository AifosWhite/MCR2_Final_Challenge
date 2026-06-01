#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from aruco_msgs.msg import MarkerArray


class JetsonArucoAdapter(Node):
    """
    Converts Jetson aruco_ros output into Karifm localisation format.

    Input:
        /marker_publisher/markers
        aruco_msgs/msg/MarkerArray

    Output:
        /aruco/detections
        std_msgs/msg/Float32MultiArray = [marker_id, distance, bearing]

    aruco_ros pose convention from camera:
        x = lateral displacement
        y = vertical displacement
        z = forward distance

    Karifm expects:
        id, distance, bearing
    """

    def __init__(self):
        super().__init__("jetson_aruco_adapter")

        self.declare_parameter("input_topic", "/marker_publisher/markers")
        self.declare_parameter("output_topic", "/aruco/detections")
        self.declare_parameter("max_detection_distance", 3.0)
        self.declare_parameter("invert_bearing", False)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.max_detection_distance = float(
            self.get_parameter("max_detection_distance").value
        )
        self.invert_bearing = bool(self.get_parameter("invert_bearing").value)

        self.sub = self.create_subscription(
            MarkerArray,
            self.input_topic,
            self.marker_callback,
            10,
        )

        self.pub = self.create_publisher(
            Float32MultiArray,
            self.output_topic,
            10,
        )

        self.get_logger().info(
            f"Jetson ArUco adapter ready: {self.input_topic} -> {self.output_topic}"
        )

    def marker_callback(self, msg: MarkerArray):
        if not msg.markers:
            return

        closest_marker = None
        closest_distance = float("inf")

        for marker in msg.markers:
            p = marker.pose.pose.position
            distance = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)

            if distance < closest_distance:
                closest_distance = distance
                closest_marker = marker

        if closest_marker is None:
            return

        p = closest_marker.pose.pose.position
        marker_id = int(closest_marker.id)

        distance = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)

        if distance > self.max_detection_distance:
            return

        bearing = math.atan2(p.x, p.z)

        if self.invert_bearing:
            bearing *= -1.0

        detection = Float32MultiArray()
        detection.data = [
            float(marker_id),
            float(distance),
            float(bearing),
        ]

        self.pub.publish(detection)

        self.get_logger().info(
            f"marker={marker_id} | distance={distance:.3f} m | "
            f"bearing={bearing:.3f} rad | raw: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = JetsonArucoAdapter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()