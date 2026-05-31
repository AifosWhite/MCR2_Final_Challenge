import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math

class ArucoMarkerBridge(Node):
    def __init__(self):
        super().__init__('aruco_marker_bridge')
        self.declare_parameter('markers_topic', '/aruco_tracker/markers')
        self.declare_parameter('detections_topic', '/aruco/detections')
        markers_topic = self.get_parameter('markers_topic').get_parameter_value().string_value
        detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value

        self.publisher = self.create_publisher(Float32MultiArray, detections_topic, 10)
        self.msg_type = None
        self._import_msg_type()
        self.subscription = self.create_subscription(
            self.msg_type, markers_topic, self.markers_callback, 10
        ) if self.msg_type else self.create_subscription(
            Float32MultiArray, markers_topic, self.generic_callback, 10
        )

    def _import_msg_type(self):
        try:
            from aruco_opencv.msg import ArucoDetection
            self.msg_type = ArucoDetection
        except ImportError:
            try:
                from aruco_msgs.msg import MarkerArray
                self.msg_type = MarkerArray
            except ImportError:
                self.get_logger().warn('No ArUco marker message type found, using generic callback')
                self.msg_type = None

    def markers_callback(self, msg):
        # Soporta ambos tipos de mensajes
        markers = []
        if hasattr(msg, 'markers'):
            for marker in msg.markers:
                marker_id = int(marker.id)
                tx = marker.pose.pose.position.x
                ty = marker.pose.pose.position.y
                tz = marker.pose.pose.position.z
                distance = math.sqrt(tx**2 + ty**2 + tz**2)
                bearing = math.atan2(tx, tz)
                markers.append((marker_id, distance, bearing))
        else:
            marker_id = int(msg.id)
            tx = msg.pose.pose.position.x
            ty = msg.pose.pose.position.y
            tz = msg.pose.pose.position.z
            distance = math.sqrt(tx**2 + ty**2 + tz**2)
            bearing = math.atan2(tx, tz)
            markers.append((marker_id, distance, bearing))
        if markers:
            closest = min(markers, key=lambda m: m[1])
            arr = Float32MultiArray()
            arr.data = [float(closest[0]), closest[1], closest[2]]
            self.publisher.publish(arr)

    def generic_callback(self, msg):
        # Si no se reconoce el tipo, solo reenvía el mensaje
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
