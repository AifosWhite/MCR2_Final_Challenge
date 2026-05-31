import math

import rclpy
from aruco_msgs.msg import MarkerArray
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ArucoMarkerBridge(Node):
    def __init__(self):
        super().__init__('aruco_marker_bridge')

        self.declare_parameter('markers_topic', '/marker_publisher/markers')
        self.declare_parameter('detections_topic', '/aruco/detections')

        markers_topic = self.get_parameter('markers_topic').value
        detections_topic = self.get_parameter('detections_topic').value

        self.create_subscription(MarkerArray, markers_topic, self.markers_callback, 10)
        self.detections_pub = self.create_publisher(Float32MultiArray, detections_topic, 10)

        self.get_logger().info(f'Bridging {markers_topic} -> {detections_topic}')

    @staticmethod
    def pose_position(marker):
        pose = marker.pose
        while hasattr(pose, 'pose'):
            pose = pose.pose
        return pose.position

    def markers_callback(self, msg):
        if not msg.markers:
            return

        detections = []
        for marker in msg.markers:
            position = self.pose_position(marker)
            distance = math.sqrt(
                position.x * position.x
                + position.y * position.y
                + position.z * position.z
            )
            bearing = math.atan2(position.x, position.z)
            detections.append((int(marker.id), distance, bearing))

        marker_id, distance, bearing = min(detections, key=lambda item: item[1])
        detection_msg = Float32MultiArray()
        detection_msg.data = [float(marker_id), float(distance), float(bearing)]
        self.detections_pub.publish(detection_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerBridge()
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
