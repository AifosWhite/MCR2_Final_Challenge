import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import math

class SimArucoNode(Node):
    def __init__(self):
        super().__init__('sim_aruco_node')
        self.declare_parameter('max_range', 2.5)
        self.declare_parameter('fov_deg', 60.0)
        self.declare_parameter('marker_ids', [70, 706, 75, 701, 703, 705, 708, 702])
        self.declare_parameter('marker_pos_x', [-1.4, -0.43, 0.868, 1.14, 0.872, -0.11, -0.408, 0.231])
        self.declare_parameter('marker_pos_y', [0.24, 0.866, 1.22, 0.242, -0.35, -1.055, -0.37, -1.31])

        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.fov_deg = self.get_parameter('fov_deg').get_parameter_value().double_value
        self.fov_rad = math.radians(self.fov_deg)
        ids = self.get_parameter('marker_ids').get_parameter_value().integer_array_value
        xs = self.get_parameter('marker_pos_x').get_parameter_value().double_array_value
        ys = self.get_parameter('marker_pos_y').get_parameter_value().double_array_value
        self.markers = list(zip(ids, xs, ys))

        self.publisher = self.create_publisher(Float32MultiArray, '/aruco/detections', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.robot_pose = (0.0, 0.0, 0.0)
        self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny, cosy)
        self.robot_pose = (x, y, theta)

    def timer_callback(self):
        x, y, theta = self.robot_pose
        visibles = []
        for marker_id, mx, my in self.markers:
            dx = mx - x
            dy = my - y
            distance = math.sqrt(dx**2 + dy**2)
            angle_to_marker = math.atan2(dy, dx)
            bearing = angle_to_marker - theta
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            if distance <= self.max_range and abs(bearing) <= self.fov_rad / 2:
                visibles.append((marker_id, distance, bearing))
        if visibles:
            closest = min(visibles, key=lambda m: m[1])
            arr = Float32MultiArray()
            arr.data = [float(closest[0]), closest[1], closest[2]]
            self.publisher.publish(arr)

def main(args=None):
    rclpy.init(args=args)
    node = SimArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
