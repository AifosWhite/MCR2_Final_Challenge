import math
import xml.etree.ElementTree as ET

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


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


class SimArucoNode(Node):
    def __init__(self):
        super().__init__('sim_aruco_node')
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 2.5)
        self.declare_parameter('fov_deg', 60.0)
        self.declare_parameter('pose_topic', 'sim_pose_odom')
        self.declare_parameter('detections_topic', '/aruco/detections')
        self.declare_parameter('world_file', '')
        self.declare_parameter('use_physical_frame', True)
        self.declare_parameter('map_width_m', 3.05)
        self.declare_parameter('map_height_m', 3.075)
        self.declare_parameter('check_occlusion', True)
        self.declare_parameter('marker_ids', [70, 75, 701, 702, 703, 705, 706, 708])
        self.declare_parameter(
            'marker_pos_x',
            [1.85, 2.75, 2.82, 0.27, 1.24, 0.89, 2.455, 1.185])
        self.declare_parameter(
            'marker_pos_y',
            [-0.30, -2.40, 0.00, -1.83, -2.07, -1.20, -1.255, -1.21])

        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.fov_deg = float(self.get_parameter('fov_deg').value)
        self.fov_rad = math.radians(self.fov_deg)
        self.use_physical_frame = bool(self.get_parameter('use_physical_frame').value)
        self.map_width_m = float(self.get_parameter('map_width_m').value)
        self.map_height_m = float(self.get_parameter('map_height_m').value)
        self.check_occlusion = bool(self.get_parameter('check_occlusion').value)
        self.walls = self.load_walls(str(self.get_parameter('world_file').value))
        marker_ids = list(self.get_parameter('marker_ids').value)
        marker_x = list(self.get_parameter('marker_pos_x').value)
        marker_y = list(self.get_parameter('marker_pos_y').value)
        self.markers = [
            (int(marker_id), float(x), float(y))
            for marker_id, x, y in zip(marker_ids, marker_x, marker_y)
        ]
        pose_topic = str(self.get_parameter('pose_topic').value)
        detections_topic = str(self.get_parameter('detections_topic').value)
        self.publisher = self.create_publisher(Float32MultiArray, detections_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, pose_topic, self.odom_callback, 10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(
            f'ArUco simulado listo: {len(self.markers)} marcadores, '
            f'FOV={self.fov_deg:.0f} deg, rango={self.max_range:.1f} m, '
            f'pose={pose_topic}, detecciones={detections_topic}.')

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
            d = math.sqrt(dx * dx + dy * dy)
            ang = math.atan2(dy, dx)
            b = ang - self.theta
            b = math.atan2(math.sin(b), math.cos(b))
            if self.min_range <= d <= self.max_range and abs(b) <= self.fov_rad / 2 and \
                    not self.is_occluded(m[1], m[2], d):
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

    def is_occluded(self, marker_x, marker_y, distance):
        if not self.check_occlusion:
            return False
        step = 0.02
        max_distance = max(0.0, distance - 0.08)
        travelled = 0.08
        while travelled < max_distance:
            t = travelled / max(distance, 1e-6)
            px = self.x + t * (marker_x - self.x)
            py = self.y + t * (marker_y - self.y)
            if self.point_hits_wall(px, py):
                return True
            travelled += step
        return False

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
        return walls or self._fallback_walls()

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
    node = SimArucoNode()
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
