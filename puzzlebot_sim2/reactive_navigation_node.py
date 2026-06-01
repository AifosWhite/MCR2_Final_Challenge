import sys

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class ReactiveNavigation(Node):
    """Bug navigation (estilo clase, probado) + ruta de waypoints en lazo.

    El nucleo de evasion es el de la clase: en 'follow_wall' se busca el objeto
    mas cercano por LiDAR y se navega manteniendose perpendicular a el
    (theta_fw = theta_ao +/- pi/2), avanzando a velocidad constante. Eso rodea
    el obstaculo sin quedarse girando en un rincon. El criterio de salida es
    Bug2 (volver a la m-line acercandose a la meta).

    Encima se agrega el secuenciado de waypoints (lazo cerrado) que pide el
    final challenge, conservando el entry point reactive_navigation_node.
    """

    def __init__(self):
        super().__init__('reactive_navigation_node')

        # --- Ruta de waypoints --------------------------------------------
        self.declare_parameter('waypoints_x', [1.20, -1.15])
        self.declare_parameter('waypoints_y', [-1.00, 1.15])
        self.declare_parameter('loop', True)

        # --- Parametros Bug (mismos nombres que el nodo de clase) ----------
        self.declare_parameter('bug_mode', 2)              # 0 / 1 / 2
        self.declare_parameter('bug_direction', 'fwcw')    # fwcw / fwccw
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('max_linear_speed', 0.12)
        self.declare_parameter('max_angular_speed', 1.8)
        self.declare_parameter('ahead_clearance_angle_deg', 35.0)
        self.declare_parameter('goal_heading_clear_angle_deg', 20.0)
        self.declare_parameter('wall_follow_safety', 0.35)
        self.declare_parameter('line_distance_threshold_bug2', 0.15)
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')

        wx = list(self.get_parameter('waypoints_x').value)
        wy = list(self.get_parameter('waypoints_y').value)
        self.waypoints = [(float(x), float(y)) for x, y in zip(wx, wy)]
        if not self.waypoints:
            self.waypoints = [(1.20, -1.00)]
        self.loop = bool(self.get_parameter('loop').value)
        self.goal_index = 0
        self.finished = False

        self.bug_mode = int(self.get_parameter('bug_mode').value)
        self.bug_direction = str(self.get_parameter('bug_direction').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.ahead_clearance_angle = np.deg2rad(
            float(self.get_parameter('ahead_clearance_angle_deg').value))
        self.goal_heading_clear_angle = np.deg2rad(
            float(self.get_parameter('goal_heading_clear_angle_deg').value))
        self.wall_follow_safety = float(self.get_parameter('wall_follow_safety').value)
        self.line_distance_threshold_bug2 = float(
            self.get_parameter('line_distance_threshold_bug2').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        # --- Estado --------------------------------------------------------
        self.lidar = LaserScan()
        self.odom_received = False
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.goal_x_current, self.goal_y_current = self.waypoints[0]

        self.state = 'go_to_goal'
        self.hit_point = None
        self.start_point = None
        self.min_goal_distance = np.inf
        self.min_line_distance = np.inf
        self.fw_commit = 0      # ciclos minimos en follow_wall (anti-chattering)

        # --- ROS -----------------------------------------------------------
        self.create_subscription(LaserScan, scan_topic, self.lidar_cb, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.robot_vel = Twist()
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            f'Bug nav listo: {len(self.waypoints)} waypoints, '
            f'bug_mode={self.bug_mode}, loop={self.loop}. '
            f'Primera meta ({self.goal_x_current:.2f}, {self.goal_y_current:.2f}).')

    # ---- Loop principal ---------------------------------------------------
    def timer_callback(self):
        if self.finished:
            self.publish_stop()
            return
        # No actuar hasta tener pose real: si no, el primer ciclo usa (0,0) y
        # dispara un follow_wall espurio apuntando mal desde el origen.
        if not self.odom_received:
            self.publish_stop()
            return
        if not self.lidar.ranges:
            self.publish_stop()
            return

        dist_to_goal, ang_to_goal = self.compute_goal_error()

        # Waypoint alcanzado -> siguiente (lazo).
        if dist_to_goal <= self.goal_tolerance:
            self.advance_goal()
            return

        if self.state == 'go_to_goal':
            if self.path_blocked(ang_to_goal):
                self.get_logger().info('Obstaculo -> follow_wall.',
                                       throttle_duration_sec=1.0)
                self.state = 'follow_wall'
                self.hit_point = (self.odom_x, self.odom_y)
                self.start_point = (self.odom_x, self.odom_y)
                self.min_goal_distance = dist_to_goal
                self.min_line_distance = self.distance_to_line()
                self.fw_commit = 10        # compromete unos ciclos al bordeo
            else:
                v, w = self.go_to_goal(dist_to_goal, ang_to_goal)
                self.publish_velocity(v, w)
                return

        if self.state == 'follow_wall':
            dist_to_goal, ang_to_goal = self.compute_goal_error()
            self.min_goal_distance = min(self.min_goal_distance, dist_to_goal)
            current_line_distance = self.distance_to_line()
            self.min_line_distance = min(self.min_line_distance, current_line_distance)

            if self.fw_commit > 0:
                self.fw_commit -= 1

            if self.fw_commit == 0 and self.can_leave_wall(
                    dist_to_goal, current_line_distance, ang_to_goal):
                self.get_logger().info('Camino despejado -> go_to_goal.',
                                       throttle_duration_sec=1.0)
                self.state = 'go_to_goal'
                v, w = self.go_to_goal(dist_to_goal, ang_to_goal)
                self.publish_velocity(v, w)
                return

            _, theta_closest = self.get_closest_object()
            v, w = self.get_theta_fw(theta_closest, self.bug_direction)
            self.publish_velocity(v, w)

    # ---- Secuenciado de waypoints ----------------------------------------
    def advance_goal(self):
        self.publish_stop()
        if self.goal_index + 1 >= len(self.waypoints):
            if self.loop:
                self.goal_index = 0
            else:
                self.finished = True
                self.get_logger().info('Ruta completa.')
                return
        else:
            self.goal_index += 1
        self.goal_x_current, self.goal_y_current = self.waypoints[self.goal_index]
        self.reset_bug_state()
        self.get_logger().info(
            f'Meta alcanzada. Siguiente WP{self.goal_index}: '
            f'({self.goal_x_current:.2f}, {self.goal_y_current:.2f}).')

    def reset_bug_state(self):
        self.state = 'go_to_goal'
        self.hit_point = None
        self.start_point = None
        self.min_goal_distance = np.inf
        self.min_line_distance = np.inf

    # ---- go_to_goal -------------------------------------------------------
    def compute_goal_error(self):
        dx = self.goal_x_current - self.odom_x
        dy = self.goal_y_current - self.odom_y
        dist = np.hypot(dx, dy)
        desired_angle = np.arctan2(dy, dx)
        ang_error = desired_angle - self.odom_theta
        ang_error = np.arctan2(np.sin(ang_error), np.cos(ang_error))
        return dist, ang_error

    def go_to_goal(self, dist, ang_error):
        if dist <= self.goal_tolerance:
            return 0.0, 0.0
        if abs(ang_error) > np.pi / 2:
            return 0.0, np.clip(2.0 * ang_error,
                                -self.max_angular_speed, self.max_angular_speed)
        v = np.clip(0.8 * dist, 0.0, self.max_linear_speed)
        w = np.clip(2.0 * ang_error, -self.max_angular_speed, self.max_angular_speed)
        if abs(ang_error) > 0.4:
            v *= 0.2
        return v, w

    # ---- Bug2: m-line -----------------------------------------------------
    def distance_to_line(self):
        if self.start_point is None:
            return np.inf
        x0, y0 = self.start_point
        x1, y1 = self.goal_x_current, self.goal_y_current
        x2, y2 = self.odom_x, self.odom_y
        dx = x1 - x0
        dy = y1 - y0
        if np.isclose(dx, 0.0) and np.isclose(dy, 0.0):
            return np.hypot(x2 - x0, y2 - y0)
        return abs(dy * x2 - dx * y2 + x1 * y0 - y1 * x0) / np.hypot(dx, dy)

    def can_leave_wall(self, dist_to_goal, line_distance, ang_to_goal):
        if not self.goal_heading_clear(ang_to_goal):
            return False
        if self.bug_mode == 2:
            return (line_distance <= self.line_distance_threshold_bug2
                    and dist_to_goal <= self.min_goal_distance)
        return dist_to_goal <= self.min_goal_distance

    # ---- Consultas al LiDAR ----------------------------------------------
    def path_blocked(self, ang_to_goal):
        return not self.is_sector_clear(ang_to_goal, self.ahead_clearance_angle)

    def goal_heading_clear(self, ang_to_goal):
        return self.is_sector_clear(ang_to_goal, self.goal_heading_clear_angle)

    def is_sector_clear(self, center_angle, half_width):
        if not self.lidar.ranges:
            return False
        min_range = np.inf
        angle = self.lidar.angle_min
        for current_range in self.lidar.ranges:
            if np.isfinite(current_range) and current_range > 0.0:
                delta = np.arctan2(np.sin(angle - center_angle),
                                   np.cos(angle - center_angle))
                if abs(delta) <= half_width:
                    min_range = min(min_range, current_range)
            angle += self.lidar.angle_increment
        return min_range > self.wall_follow_safety

    def get_closest_object(self):
        # Robusto ante inf/0 del scan simulado (la version de clase asumia
        # rangos finitos del robot real).
        best_r = np.inf
        best_i = -1
        angle_inc = self.lidar.angle_increment
        for i, r in enumerate(self.lidar.ranges):
            if np.isfinite(r) and r > 0.0 and r < best_r:
                best_r = r
                best_i = i
        if best_i < 0:
            return np.inf, 0.0
        theta = self.lidar.angle_min + best_i * angle_inc
        return best_r, np.arctan2(np.sin(theta), np.cos(theta))

    def get_theta_ao(self, theta_closest):
        theta_ao = theta_closest + np.pi
        return np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

    def get_theta_fw(self, theta_closest, direction='fwcw'):
        theta_ao = self.get_theta_ao(theta_closest)
        if direction == 'fwccw':
            theta_fw = theta_ao + np.pi / 2
        else:
            theta_fw = theta_ao - np.pi / 2
        theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))
        v = self.max_linear_speed * 0.8
        w = np.clip(1.1 * theta_fw, -self.max_angular_speed, self.max_angular_speed)
        return v, w

    # ---- Callbacks --------------------------------------------------------
    def lidar_cb(self, msg):
        self.lidar = msg

    def odom_cb(self, msg):
        self.odom_received = True
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_theta = np.arctan2(siny, cosy)

    def goal_cb(self, msg):
        # Override externo (RViz 2D Nav Goal): reemplaza la ruta por una meta.
        self.waypoints = [(float(msg.pose.position.x), float(msg.pose.position.y))]
        self.goal_index = 0
        self.goal_x_current, self.goal_y_current = self.waypoints[0]
        self.finished = False
        self.reset_bug_state()
        self.get_logger().info(
            f'Nueva meta: ({self.goal_x_current:.2f}, {self.goal_y_current:.2f}).')

    def publish_velocity(self, v, w):
        self.robot_vel.linear.x = float(v)
        self.robot_vel.angular.z = float(w)
        self.cmd_vel_pub.publish(self.robot_vel)

    def publish_stop(self):
        self.publish_velocity(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
