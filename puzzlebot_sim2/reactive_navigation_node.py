#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ReactiveNavigationNode(Node):
    def __init__(self):
        super().__init__('reactive_navigation_node')

        self.declare_parameter('bug_algorithm', 2)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.bug_algorithm = int(self.get_parameter('bug_algorithm').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.dt = 0.05

        self.waypoints = [
            (0.6, 1.2),
            (1.8, 1.2),
            (1.8, 2.4),
            (2.4, 2.4),
        ]

        self.wp_index = 0
        self.state = 'go_to_goal'

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_ready = False
        self.scan: Optional[LaserScan] = None

        self.m_start = (0.0, 0.0)
        self.m_line_ready = False
        self.hit_distance = math.inf
        self.wall_start_time = None

        self.goal_tolerance = 0.13
        self.max_v = 0.16
        self.max_w = 1.30

        # Umbral de 30 cm para no rozar paredes con EKF imperfecto.
        self.block_distance = 0.30
        self.clear_distance = 0.38
        self.wall_distance = 0.30
        self.m_line_tolerance = 0.10
        self.leave_improvement = 0.06
        self.min_wall_time = 0.8

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(f'Reactive navigation ready: Bug {self.bug_algorithm}')

    @staticmethod
    def wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.x = p.x
        self.y = p.y
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.odom_ready = True

    def scan_callback(self, msg: LaserScan):
        self.scan = msg

    def control_loop(self):
        if not self.odom_ready or self.scan is None:
            self.publish_cmd(0.0, 0.0)
            return

        if not self.m_line_ready:
            self.start_new_m_line()

        if self.wp_index >= len(self.waypoints):
            self.publish_cmd(0.0, 0.0)
            return

        distance, heading_error = self.goal_error()

        if distance < self.goal_tolerance:
            self.advance_waypoint()
            return

        if self.state == 'go_to_goal':
            if self.path_blocked(heading_error):
                self.state = 'follow_wall'
                self.hit_distance = distance
                self.wall_start_time = self.get_clock().now()
                self.get_logger().info('Pared detectada: cambio a wall-following')
            else:
                v, w = self.go_to_goal(distance, heading_error)
                self.publish_cmd(v, w)
                return

        if self.state == 'follow_wall':
            if self.can_leave_wall(distance, heading_error):
                self.state = 'go_to_goal'
                self.get_logger().info('Camino libre: regreso a go-to-goal')
                v, w = self.go_to_goal(distance, heading_error)
            else:
                v, w = self.follow_right_wall()

            self.publish_cmd(v, w)

    def advance_waypoint(self):
        self.get_logger().info(f'Waypoint {self.wp_index + 1}/4 alcanzado')
        self.wp_index += 1
        self.publish_cmd(0.0, 0.0)

        if self.wp_index < len(self.waypoints):
            self.state = 'go_to_goal'
            self.start_new_m_line()
        else:
            self.get_logger().info('Ruta de 4 waypoints terminada')

    def start_new_m_line(self):
        self.m_start = (self.x, self.y)
        self.m_line_ready = True
        self.hit_distance = math.inf
        self.wall_start_time = None

    def goal_error(self) -> Tuple[float, float]:
        gx, gy = self.waypoints[self.wp_index]
        dx = gx - self.x
        dy = gy - self.y
        distance = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        return distance, self.wrap(desired_yaw - self.yaw)

    def go_to_goal(self, distance: float, heading_error: float) -> Tuple[float, float]:
        k_v = 0.45
        k_w = 1.8

        w = self.clamp(k_w * heading_error, self.max_w)
        v = min(k_v * distance, self.max_v)

        if abs(heading_error) > 0.45:
            v *= 0.35

        return v, w

    def follow_right_wall(self) -> Tuple[float, float]:
        front = self.sector_min(0.0, math.radians(18.0))
        right = self.sector_min(-math.pi / 2.0, math.radians(22.0))
        front_right = self.sector_min(-math.pi / 4.0, math.radians(18.0))

        if front < self.block_distance:
            return 0.04, 0.95

        if not math.isfinite(right):
            return 0.10, -0.45

        k_wall = 2.0
        error = self.wall_distance - right
        w = k_wall * error

        if front_right < self.wall_distance:
            w += 0.35

        v = 0.11
        return v, self.clamp(w, self.max_w)

    def path_blocked(self, heading_error: float) -> bool:
        return self.sector_min(heading_error, math.radians(18.0)) < self.block_distance

    def path_clear_to_goal(self, heading_error: float) -> bool:
        return self.sector_min(heading_error, math.radians(22.0)) > self.clear_distance

    def can_leave_wall(self, distance: float, heading_error: float) -> bool:
        if not self.wall_time_ok():
            return False

        if not self.path_clear_to_goal(heading_error):
            return False

        if self.bug_algorithm == 0:
            return True

        closer_than_hit = distance < (self.hit_distance - self.leave_improvement)
        return closer_than_hit and self.distance_to_m_line() < self.m_line_tolerance

    def wall_time_ok(self) -> bool:
        if self.wall_start_time is None:
            return True

        elapsed = (self.get_clock().now() - self.wall_start_time).nanoseconds * 1e-9
        return elapsed > self.min_wall_time

    def distance_to_m_line(self) -> float:
        gx, gy = self.waypoints[self.wp_index]
        x0, y0 = self.m_start
        dx = gx - x0
        dy = gy - y0

        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return math.hypot(self.x - x0, self.y - y0)

        num = abs(dy * self.x - dx * self.y + gx * y0 - gy * x0)
        den = math.hypot(dx, dy)
        return num / den

    def sector_min(self, center: float, half_width: float) -> float:
        best = math.inf
        angle = self.scan.angle_min

        for r in self.scan.ranges:
            if math.isfinite(r) and self.scan.range_min < r < self.scan.range_max:
                if abs(self.wrap(angle - center)) <= half_width:
                    best = min(best, r)
            angle += self.scan.angle_increment

        return best

    def publish_cmd(self, v: float, w: float):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveNavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
