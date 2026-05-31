#!/usr/bin/env python3

import math
import xml.etree.ElementTree as ET
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SimLidarNode(Node):
    def __init__(self):
        super().__init__('sim_lidar_node')

        self.declare_parameter('world_file', '')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('pose_topic', '/pose_sim')

        world_file = str(self.get_parameter('world_file').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        pose_topic = str(self.get_parameter('pose_topic').value)

        self.pose: Optional[Tuple[float, float, float]] = None
        self.walls = self.load_walls(world_file)

        self.range_min = 0.05
        self.range_max = 3.5
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = math.radians(1.0)

        self.scan_pub = self.create_publisher(LaserScan, scan_topic, 10)
        self.create_subscription(PoseStamped, pose_topic, self.pose_callback, 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(f'Sim LiDAR ready with {len(self.walls)} wall boxes')

    @staticmethod
    def load_walls(world_file: str) -> List[Tuple[float, float, float, float, float]]:
        if not world_file:
            return []

        root = ET.parse(world_file).getroot()
        walls = []

        for link in root.findall(".//model[@name='newmap']/link"):
            pose = link.findtext('pose')
            size = link.findtext('./collision/geometry/box/size')
            if pose is None or size is None:
                continue

            px, py, _, _, _, yaw = [float(v) for v in pose.split()]
            sx, sy, _ = [float(v) for v in size.split()]
            walls.append((px, py, yaw, sx, sy))

        return walls

    def pose_callback(self, msg: PoseStamped):
        q = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.pose = (msg.pose.position.x, msg.pose.position.y, yaw)

    def timer_callback(self):
        if self.pose is None:
            return

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.05
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        rays = int(round((self.angle_max - self.angle_min) / self.angle_increment)) + 1
        msg.ranges = [self.cast_ray(self.angle_min + i * self.angle_increment) for i in range(rays)]
        self.scan_pub.publish(msg)

    def cast_ray(self, local_angle: float) -> float:
        x, y, yaw = self.pose
        angle = yaw + local_angle
        dx = math.cos(angle)
        dy = math.sin(angle)
        best = self.range_max

        for wall in self.walls:
            hit = self.ray_box_distance(x, y, dx, dy, wall)
            if hit is not None:
                best = min(best, hit)

        return best

    @staticmethod
    def ray_box_distance(
        x: float,
        y: float,
        dx: float,
        dy: float,
        wall: Tuple[float, float, float, float, float],
    ) -> Optional[float]:
        cx, cy, yaw, sx, sy = wall
        c = math.cos(-yaw)
        s = math.sin(-yaw)

        ox = c * (x - cx) - s * (y - cy)
        oy = s * (x - cx) + c * (y - cy)
        rx = c * dx - s * dy
        ry = s * dx + c * dy

        t_min = -math.inf
        t_max = math.inf

        for origin, direction, limit in ((ox, rx, sx / 2.0), (oy, ry, sy / 2.0)):
            if abs(direction) < 1e-9:
                if abs(origin) > limit:
                    return None
                continue

            t1 = (-limit - origin) / direction
            t2 = (limit - origin) / direction
            t_min = max(t_min, min(t1, t2))
            t_max = min(t_max, max(t1, t2))

            if t_min > t_max:
                return None

        if t_max < 0.0:
            return None

        return max(t_min, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = SimLidarNode()
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
