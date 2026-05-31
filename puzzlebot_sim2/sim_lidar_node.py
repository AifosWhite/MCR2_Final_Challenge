import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


WALLS = [
    (-1.640, 0.000, 0.150, 3.405, -1.570),
    (1.640, 0.000, 0.150, 3.406, 1.570),
    (0.002, 1.627, 3.430, 0.150, -3.141),
    (0.000, -1.628, 3.426, 0.150, 0.000),
    (-1.304, 0.318, 0.753, 0.100, 0.000),
    (-0.978, 0.631, 0.100, 0.726, 1.569),
    (-0.334, 0.944, 1.386, 0.100, 0.000),
    (0.309, 0.632, 0.100, 0.725, -1.571),
    (0.947, 1.251, 0.100, 0.688, 1.572),
    (-0.979, -1.292, 0.100, 0.745, 1.571),
    (1.241, 0.320, 0.675, 0.100, -0.003),
    (0.951, -0.320, 0.100, 1.381, -1.571),
    (0.611, -0.323, 0.700, 0.100, 3.142),
    (0.309, -1.277, 0.100, 0.700, -1.565),
    (-0.010, -0.977, 0.741, 0.100, -3.141),
    (-0.330, -0.336, 0.100, 1.382, 1.571),
    (-0.700, -0.323, 0.700, 0.100, 0.000),
]


class SimLidar(Node):
    def __init__(self):
        super().__init__('sim_lidar_node')
        self.declare_parameter('range_max', 3.0)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('samples', 181)
        self.declare_parameter('update_rate', 10.0)

        self.range_max = float(self.get_parameter('range_max').value)
        self.angle_min = float(self.get_parameter('angle_min').value)
        self.angle_max = float(self.get_parameter('angle_max').value)
        self.samples = int(self.get_parameter('samples').value)
        self.angle_increment = (self.angle_max - self.angle_min) / (self.samples - 1)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_timer(1.0 / float(self.get_parameter('update_rate').value), self.publish_scan)
        self.get_logger().info('LiDAR simulado listo: publica /scan desde el mapa del laberinto.')

    def odom_callback(self, msg):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny, cosy)

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_footprint'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.05
        scan.range_max = self.range_max
        scan.ranges = []

        for i in range(self.samples):
            angle = self.theta + self.angle_min + i * self.angle_increment
            scan.ranges.append(float(self.cast_ray(angle)))

        self.scan_pub.publish(scan)

    def cast_ray(self, angle):
        dx = math.cos(angle)
        dy = math.sin(angle)
        best = self.range_max
        step = 0.02
        distance = scan_min = 0.05
        while distance <= self.range_max:
            px = self.x + distance * dx
            py = self.y + distance * dy
            if self.point_hits_wall(px, py):
                best = distance
                break
            distance += step
        return max(scan_min, best)

    def point_hits_wall(self, px, py):
        for cx, cy, sx, sy, yaw in WALLS:
            c = math.cos(-yaw)
            s = math.sin(-yaw)
            lx = c * (px - cx) - s * (py - cy)
            ly = s * (px - cx) + c * (py - cy)
            if abs(lx) <= sx / 2.0 and abs(ly) <= sy / 2.0:
                return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = SimLidar()
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
