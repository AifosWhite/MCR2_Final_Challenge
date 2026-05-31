
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import math


class SimArucoNode(Node):
    def __init__(self):
        super().__init__('sim_aruco_node')
        self.max_range = 2.5
        self.fov_deg = 60.0
        self.fov_rad = math.radians(self.fov_deg)
        self.marker_ids = [70, 706, 75, 701, 703, 705, 708, 702]
        self.marker_x = [-1.300, -0.780, 0.830, -1.565, 0.560, -0.230, -0.270, 0.365]
        self.marker_y = [0.252, 0.820, 1.120, -0.420, -0.205, -0.880, -0.250, -1.150]
        self.markers = []
        for i in range(len(self.marker_ids)):
            self.markers.append((self.marker_ids[i], self.marker_x[i], self.marker_y[i]))
        self.publisher = self.create_publisher(Float32MultiArray, '/aruco/detections', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny, cosy)

    def timer_callback(self):
        visibles = []
        for m in self.markers:
            dx = m[1] - self.x
            dy = m[2] - self.y
            d = math.sqrt(dx*dx + dy*dy)
            ang = math.atan2(dy, dx)
            b = ang - self.theta
            b = math.atan2(math.sin(b), math.cos(b))
            if d <= self.max_range and abs(b) <= self.fov_rad/2:
                visibles.append((m[0], d, b))
        if len(visibles) > 0:
            min_idx = 0
            for i in range(1, len(visibles)):
                if visibles[i][1] < visibles[min_idx][1]:
                    min_idx = i
            v = visibles[min_idx]
            arr = Float32MultiArray()
            arr.data = [float(v[0]), v[1], v[2]]
            self.publisher.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = SimArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
