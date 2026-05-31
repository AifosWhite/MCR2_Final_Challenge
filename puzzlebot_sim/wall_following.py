import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped

import numpy as np
import signal
import sys


class LaserScanSub(Node):
    def __init__(self):
        super().__init__('avoid_obstacle')

        # Parameters
        self.declare_parameter('bug_mode', 0)
        self.declare_parameter('bug_direction', 'fwcw')
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('goal_topic', 'current_goal')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('use_static_goal', False)
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('max_linear_speed', 0.25)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('ahead_clearance_angle_deg', 25.0)
        self.declare_parameter('goal_heading_clear_angle_deg', 20.0)
        self.declare_parameter('wall_follow_safety', 0.25)
        self.declare_parameter('line_distance_threshold_bug2', 0.15)

        self.bug_mode = int(self.get_parameter('bug_mode').value)
        self.bug_direction = str(self.get_parameter('bug_direction').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.use_static_goal = bool(self.get_parameter('use_static_goal').value)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.ahead_clearance_angle = np.deg2rad(float(self.get_parameter('ahead_clearance_angle_deg').value))
        self.goal_heading_clear_angle = np.deg2rad(float(self.get_parameter('goal_heading_clear_angle_deg').value))
        self.wall_follow_safety = float(self.get_parameter('wall_follow_safety').value)
        self.line_distance_threshold_bug2 = float(self.get_parameter('line_distance_threshold_bug2').value)

        # Robot state
        self.lidar = LaserScan()
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.goal_x_current = self.goal_x
        self.goal_y_current = self.goal_y
        self.goal_received = False
        self.state = 'go_to_goal'
        self.hit_point = None
        self.start_point = None  # Punto inicial cuando se golpea obstáculo (para Bug 2)
        self.min_goal_distance = np.inf
        self.min_line_distance = np.inf

        # Subscriptions and publishers
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, qos_profile_sensor_data)
        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self.goal_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.robot_vel = Twist()
        self.timer = self.create_timer(0.1, self.timer_callback)

        signal.signal(signal.SIGINT, self.shutdown_function)
        self.get_logger().info(f'Node initialized: bug_mode={self.bug_mode}, direction={self.bug_direction}')

    def timer_callback(self):
        if self.use_static_goal and not self.goal_received:
            self.goal_x_current = self.goal_x
            self.goal_y_current = self.goal_y
            self.goal_received = True

        if not self.goal_received:
            self.get_logger().debug('No goal received yet.')
            self.publish_stop()
            return

        if not self.lidar.ranges:
            self.get_logger().debug('Waiting for lidar data...')
            self.publish_stop()
            return

        dist_to_goal, ang_to_goal = self.compute_goal_error()

        if dist_to_goal <= self.goal_tolerance:
            self.get_logger().info('Goal reached.')
            self.publish_stop()
            return

        if self.state == 'go_to_goal':
            if self.path_blocked(ang_to_goal):
                self.get_logger().info('Obstacle detected, switching to wall following.')
                self.state = 'follow_wall'
                self.hit_point = (self.odom_x, self.odom_y)
                self.start_point = (self.odom_x, self.odom_y)  # Guardar punto inicial para Bug 2
                self.min_goal_distance = dist_to_goal
                self.min_line_distance = self.distance_to_line()
            else:
                v, w = self.go_to_goal(dist_to_goal, ang_to_goal)
                self.publish_velocity(v, w)
                return

        if self.state == 'follow_wall':
            dist_to_goal, ang_to_goal = self.compute_goal_error()
            self.min_goal_distance = min(self.min_goal_distance, dist_to_goal)
            current_line_distance = self.distance_to_line()
            self.min_line_distance = min(self.min_line_distance, current_line_distance)

            if self.can_leave_wall(dist_to_goal, current_line_distance, ang_to_goal):
                self.get_logger().info('Resuming go-to-goal.')
                self.state = 'go_to_goal'
                v, w = self.go_to_goal(dist_to_goal, ang_to_goal)
                self.publish_velocity(v, w)
                return

            closest_range, theta_closest = self.get_closest_object()
            v, w = self.get_theta_fw(theta_closest, self.bug_direction)
            self.publish_velocity(v, w)

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
            return 0.0, np.clip(2.0 * ang_error, -self.max_angular_speed, self.max_angular_speed)

        v = np.clip(0.8 * dist, 0.0, self.max_linear_speed)
        w = np.clip(2.0 * ang_error, -self.max_angular_speed, self.max_angular_speed)
        if abs(ang_error) > 0.4:
            v *= 0.2
        return v, w

    def distance_to_line(self):
        """Calcula distancia desde posición actual a la línea start-goal.
        
        Para Bug 2, usa la línea inicial desde start_point al goal_x_current/goal_y_current.
        Esta línea permanece fija durante toda la fase de wall following.
        """
        if self.start_point is None:
            return np.inf

        # Línea de referencia: desde start_point al goal actual
        x0, y0 = self.start_point
        x1, y1 = self.goal_x_current, self.goal_y_current
        x2, y2 = self.odom_x, self.odom_y

        dx = x1 - x0
        dy = y1 - y0
        if np.isclose(dx, 0.0) and np.isclose(dy, 0.0):
            return np.hypot(x2 - x0, y2 - y0)

        return abs(dy * x2 - dx * y2 + x1 * y0 - y1 * x0) / np.hypot(dx, dy)

    def can_leave_wall(self, dist_to_goal, line_distance, ang_to_goal):
        """Decide si el robot puede dejar de seguir la pared.
        
        Para Bug 2: usa umbral fijo (line_distance_threshold_bug2) para determinar
        si está "sobre la línea start-goal", en lugar de comparar con el mínimo.
        """
        if not self.goal_heading_clear(ang_to_goal):
            return False

        if self.bug_mode == 2:
            # Usa umbral fijo: estamos sobre la línea si distance_to_line <= threshold
            return line_distance <= self.line_distance_threshold_bug2 and dist_to_goal <= self.min_goal_distance
        return dist_to_goal <= self.min_goal_distance

    def path_blocked(self, ang_to_goal):
        return not self.is_sector_clear(ang_to_goal, self.ahead_clearance_angle)

    def goal_heading_clear(self, ang_to_goal):
        return self.is_sector_clear(ang_to_goal, self.goal_heading_clear_angle)

    def is_sector_clear(self, center_angle, half_width):
        if not self.lidar.ranges:
            return False

        min_range = np.inf
        angle = self.lidar.angle_min
        for r in self.lidar.ranges:
            if np.isfinite(r):
                delta = np.arctan2(np.sin(angle - center_angle), np.cos(angle - center_angle))
                if abs(delta) <= half_width:
                    min_range = min(min_range, r)
            angle += self.lidar.angle_increment

        return min_range > self.wall_follow_safety

    def get_closest_object(self):
        closest_range = min(self.lidar.ranges)
        closest_index = self.lidar.ranges.index(closest_range)
        theta_closest = self.lidar.angle_min + closest_index * self.lidar.angle_increment
        theta_closest = np.arctan2(np.sin(theta_closest), np.cos(theta_closest))
        return closest_range, theta_closest

    def get_theta_ao(self, theta_closest):
        theta_ao = theta_closest + np.pi
        return np.arctan2(np.sin(theta_ao), np.cos(theta_ao))

    def get_theta_fw(self, theta_ao, direction='fwcw'):
        theta_ao = self.get_theta_ao(theta_ao)
        if direction == 'fwccw':
            theta_fw = theta_ao + np.pi / 2
        else:
            theta_fw = theta_ao - np.pi / 2
        theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))
        v = self.max_linear_speed * 0.8
        w = np.clip(1.9 * theta_fw, -self.max_angular_speed, self.max_angular_speed)
        return v, w

    def lidar_cb(self, lidar_msg):
        self.lidar = lidar_msg

    def odom_cb(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_theta = np.arctan2(siny, cosy)

    def goal_cb(self, msg):
        self.goal_x_current = msg.pose.position.x
        self.goal_y_current = msg.pose.position.y
        self.goal_received = True
        self.state = 'go_to_goal'
        self.hit_point = None
        self.start_point = None  # Resetear cuando se recibe nueva meta
        self.min_goal_distance = np.inf
        self.min_line_distance = np.inf
        self.get_logger().info(f'New goal received: ({self.goal_x_current:.2f}, {self.goal_y_current:.2f})')

    def publish_velocity(self, v, w):
        self.robot_vel.linear.x = float(v)
        self.robot_vel.angular.z = float(w)
        self.cmd_vel_pub.publish(self.robot_vel)

    def publish_stop(self):
        self.publish_velocity(0.0, 0.0)

    def shutdown_function(self, signum, frame):
        self.get_logger().info('Shutting down. Stopping robot...')
        self.publish_stop()
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()