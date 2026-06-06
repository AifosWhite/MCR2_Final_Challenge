import math
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


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
        self.declare_parameter('world_file', '')
        self.declare_parameter('range_max', 3.0)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('samples', 360)
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('use_physical_frame', True)
        self.declare_parameter('map_width_m', 3.05)
        self.declare_parameter('map_height_m', 3.075)

        self.range_max = float(self.get_parameter('range_max').value)
        self.angle_min = float(self.get_parameter('angle_min').value)
        self.angle_max = float(self.get_parameter('angle_max').value)
        self.samples = int(self.get_parameter('samples').value)
        self.angle_increment = (self.angle_max - self.angle_min) / (self.samples - 1)
        self.use_physical_frame = bool(self.get_parameter('use_physical_frame').value)
        self.map_width_m = float(self.get_parameter('map_width_m').value)
        self.map_height_m = float(self.get_parameter('map_height_m').value)
        self.walls = self.load_walls(str(self.get_parameter('world_file').value))
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.walls_pub = self.create_publisher(MarkerArray, 'sim_lidar_walls', 1)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_timer(1.0 / float(self.get_parameter('update_rate').value), self.publish_scan)
        self.create_timer(1.0, self.publish_walls)
        frame_name = 'fisico esquina' if self.use_physical_frame else 'Gazebo centrado'
        self.get_logger().info(
            f'LiDAR simulado listo: publica /scan con {len(self.walls)} paredes '
            f'en frame {frame_name}.')

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

    def publish_walls(self):
        markers = MarkerArray()
        for index, (cx, cy, sx, sy, yaw) in enumerate(self.walls):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'
            marker.ns = 'sim_lidar_walls'
            marker.id = index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.025
            marker.color.r = 0.0
            marker.color.g = 0.85
            marker.color.b = 1.0
            marker.color.a = 0.9

            c = math.cos(yaw)
            s = math.sin(yaw)
            corners = [
                (-sx / 2.0, -sy / 2.0),
                (sx / 2.0, -sy / 2.0),
                (sx / 2.0, sy / 2.0),
                (-sx / 2.0, sy / 2.0),
                (-sx / 2.0, -sy / 2.0),
            ]
            for lx, ly in corners:
                point = Point()
                point.x = float(cx + c * lx - s * ly)
                point.y = float(cy + s * lx + c * ly)
                point.z = 0.08
                marker.points.append(point)
            markers.markers.append(marker)
        self.walls_pub.publish(markers)

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
        for cx, cy, sx, sy, yaw in self.walls:
            c = math.cos(-yaw)
            s = math.sin(-yaw)
            lx = c * (px - cx) - s * (py - cy)
            ly = s * (px - cx) + c * (py - cy)
            if abs(lx) <= sx / 2.0 and abs(ly) <= sy / 2.0:
                return True
        return False

    def load_walls(self, world_file):
        if not world_file:
            return self._fallback_walls()

        try:
            root = ET.parse(world_file).getroot()
        except (OSError, ET.ParseError) as exc:
            self.get_logger().warn(f'No pude leer {world_file}; uso paredes fallback. {exc}')
            return self._fallback_walls()

        map_model = None
        for model in root.findall('.//model'):
            if model.get('name') == 'mapcorreg':
                map_model = model
                break
        if map_model is None:
            return self._fallback_walls()

        model_pose = self.parse_pose(map_model.findtext('pose'))
        walls = []
        for link in map_model.findall('.//link'):
            collision = link.find('collision')
            if collision is None:
                continue
            size_text = collision.findtext('geometry/box/size')
            if not size_text:
                continue

            size = [float(value) for value in size_text.split()]
            if len(size) < 2:
                continue

            link_pose = self.parse_pose(link.findtext('pose'))
            wx, wy, wyaw = self.compose_pose(model_pose, link_pose)
            if self.use_physical_frame:
                wx, wy, wyaw = self.centered_to_physical(wx, wy, wyaw)
            walls.append((wx, wy, size[0], size[1], wyaw))

        if walls:
            return walls
        return self._fallback_walls()

    def _fallback_walls(self):
        if self.use_physical_frame:
            converted = []
            for cx, cy, sx, sy, yaw in WALLS:
                wx, wy, wyaw = self.centered_to_physical(cx, cy, yaw)
                converted.append((wx, wy, sx, sy, wyaw))
            return converted
        return WALLS

    def centered_to_physical(self, x, y, yaw):
        physical_x = y + self.map_height_m / 2.0
        physical_y = -(x + self.map_width_m / 2.0)
        physical_yaw = math.atan2(-math.cos(yaw), math.sin(yaw))
        return physical_x, physical_y, physical_yaw

    def parse_pose(self, text):
        if not text:
            return 0.0, 0.0, 0.0
        values = [float(value) for value in text.split()]
        while len(values) < 6:
            values.append(0.0)
        return values[0], values[1], values[5]

    def compose_pose(self, parent, child):
        px, py, pyaw = parent
        cx, cy, cyaw = child
        c = math.cos(pyaw)
        s = math.sin(pyaw)
        return px + c * cx - s * cy, py + s * cx + c * cy, pyaw + cyaw


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
