import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math


class ArucoMarkerBridge(Node):
    def __init__(self):
        super().__init__('aruco_marker_bridge')
        self.markers_topic = '/aruco_tracker/markers'
        self.detections_topic = '/aruco/detections'
        self.publisher = self.create_publisher(Float32MultiArray, self.detections_topic, 10)
        self.subscription = self.create_subscription(Float32MultiArray, self.markers_topic, self.generic_callback, 10)

    def generic_callback(self, msg):
        # Espera un Float32MultiArray con [id, x, y, z]
        if len(msg.data) >= 4:
            marker_id = int(msg.data[0])
            tx = msg.data[1]
            ty = msg.data[2]
            tz = msg.data[3]
            d = math.sqrt(tx*tx + ty*ty + tz*tz)
            b = math.atan2(tx, tz)
            arr = Float32MultiArray()
            arr.data = [float(marker_id), d, b]
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
