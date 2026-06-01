import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ReactiveNavigation(Node):
    def __init__(self):
        super().__init__('reactive_navigation_node')

        self.declare_parameter('waypoints_x', [1.15, 1.15, -1.15, -1.15])
        self.declare_parameter('waypoints_y', [-1.15, 1.15, 1.15, -1.15])
        self.declare_parameter('goal_tolerance', 0.12)
        self.declare_parameter('max_linear_speed', 0.14)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('wall_distance', 0.35)
        self.declare_parameter('front_clearance', 0.45)
        self.declare_parameter('side_clearance', 0.22)
        self.declare_parameter('emergency_stop_distance', 0.20)
        self.declare_parameter('wall_acquire_distance', 0.75)
        self.declare_parameter('wall_leave_clearance', 0.60)
        self.declare_parameter('bug_algorithm', 2)

        wx = list(self.get_parameter('waypoints_x').value)
        wy = list(self.get_parameter('waypoints_y').value)
        self.waypoints = [(float(x), float(y)) for x, y in zip(wx, wy)]
        if not self.waypoints:
            self.waypoints = [(1.15, -1.15)]
        self.goal_index = 0
        self.goal_x, self.goal_y = self.waypoints[self.goal_index]
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.max_v = float(self.get_parameter('max_linear_speed').value)
        self.max_w = float(self.get_parameter('max_angular_speed').value)
        self.wall_distance = float(self.get_parameter('wall_distance').value)
        self.front_clearance = float(self.get_parameter('front_clearance').value)
        self.side_clearance = float(self.get_parameter('side_clearance').value)
        self.emergency_stop_distance = float(self.get_parameter('emergency_stop_distance').value)
        self.wall_acquire_distance = float(self.get_parameter('wall_acquire_distance').value)
        self.wall_leave_clearance = float(self.get_parameter('wall_leave_clearance').value)
        self.bug_algorithm = int(self.get_parameter('bug_algorithm').value)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.scan = None
        self.state = 'go_to_goal'
        self.wall_side = 'right'
        self.wall_lock_count = 0
        self.hit_point = None
        self.best_goal_distance = math.inf

        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'Navegacion Bug lista hacia ({self.goal_x:.2f}, {self.goal_y:.2f}).')

    def odom_callback(self, msg):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny, cosy)

    def scan_callback(self, msg):
        self.scan = msg

    def goal_callback(self, msg):
        self.goal_x = float(msg.pose.position.x)
        self.goal_y = float(msg.pose.position.y)
        self.waypoints = [(self.goal_x, self.goal_y)]
        self.goal_index = 0
        self.state = 'go_to_goal'
        self.hit_point = None
        self.best_goal_distance = math.inf
        self.get_logger().info(f'Nueva meta: ({self.goal_x:.2f}, {self.goal_y:.2f}).')

    def control_loop(self):
        if self.scan is None:
            self.publish_cmd(0.0, 0.0)
            return

        dist, angle_error = self.goal_error()
        if dist < self.goal_tolerance:
            self.next_goal()
            return

        if self.collision_guard():
            return

        self.state = 'go_to_goal'
        self.go_to_goal(dist, angle_error)

    def collision_guard(self):
        front = self.range_in_sector(0.0, math.radians(25.0))
        front_left = self.range_in_sector(math.pi / 4.0, math.radians(20.0))
        front_right = self.range_in_sector(-math.pi / 4.0, math.radians(20.0))
        left = self.range_in_sector(math.pi / 2.0, math.radians(25.0))
        right = self.range_in_sector(-math.pi / 2.0, math.radians(25.0))

        if front < self.front_clearance:
            turn_left = right < left
            turn_speed = 0.40 if front < self.emergency_stop_distance else 0.28
            self.publish_cmd(0.0, turn_speed if turn_left else -turn_speed)
            self.state = 'avoid_obstacle'
            self.wall_side = 'right' if turn_left else 'left'
            self.wall_lock_count = 10
            return True

        if front_right < self.side_clearance or right < self.side_clearance:
            self.publish_cmd(0.012, 0.28)
            self.state = 'avoid_obstacle'
            self.wall_side = 'right'
            self.wall_lock_count = 10
            return True

        if front_left < self.side_clearance or left < self.side_clearance:
            self.publish_cmd(0.012, -0.28)
            self.state = 'avoid_obstacle'
            self.wall_side = 'left'
            self.wall_lock_count = 10
            return True

        return False

    def goal_error(self):
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.hypot(dx, dy)
        desired = math.atan2(dy, dx)
        error = desired - self.theta
        error = math.atan2(math.sin(error), math.cos(error))
        return dist, error

    def next_goal(self):
        self.publish_cmd(0.0, 0.0)
        self.goal_index = (self.goal_index + 1) % len(self.waypoints)
        self.goal_x, self.goal_y = self.waypoints[self.goal_index]
        self.state = 'go_to_goal'
        self.wall_side = 'right'
        self.wall_lock_count = 0
        self.hit_point = None
        self.best_goal_distance = math.inf
        self.get_logger().info(
            f'Meta alcanzada. Siguiente WP{self.goal_index}: '
            f'({self.goal_x:.2f}, {self.goal_y:.2f}).'
        )

    def go_to_goal(self, dist, angle_error):
        w = float(np.clip(2.0 * angle_error, -self.max_w, self.max_w))
        v = float(np.clip(0.8 * dist, 0.0, self.max_v))
        if abs(angle_error) > 0.35:
            v = 0.0
        front = self.range_in_sector(0.0, math.radians(25.0))
        if front < self.front_clearance * 1.5:
            v = min(v, 0.03)
        self.publish_cmd(v, w)

    def follow_wall(self, dist, angle_error):
        if self.wall_lock_count > 0:
            self.wall_lock_count -= 1

        front = self.range_in_sector(0.0, math.radians(20.0))
        left = self.range_in_sector(math.pi / 2.0, math.radians(20.0))
        right = self.range_in_sector(-math.pi / 2.0, math.radians(25.0))
        front_left = self.range_in_sector(math.pi / 4.0, math.radians(20.0))
        front_right = self.range_in_sector(-math.pi / 4.0, math.radians(20.0))
        self.best_goal_distance = min(self.best_goal_distance, dist)

        if self.can_leave_wall(dist, angle_error):
            self.state = 'go_to_goal'
            self.wall_lock_count = 0
            self.go_to_goal(dist, angle_error)
            return

        diagonal = front_left if self.wall_side == 'left' else front_right

        if front < self.front_clearance:
            v = 0.0
            w = -0.38 if self.wall_side == 'left' else 0.38
        elif diagonal < self.wall_distance * 0.75:
            v = 0.015
            w = -0.42 if self.wall_side == 'left' else 0.42
        else:
            if self.wall_side == 'left':
                side_range = left
                diagonal_range = front_left
                turn_sign = -1.0
            else:
                side_range = right
                diagonal_range = front_right
                turn_sign = 1.0

            if not math.isfinite(side_range) or side_range > self.wall_acquire_distance:
                v = 0.025
                w = -0.35 * turn_sign
            elif diagonal_range < self.side_clearance:
                v = 0.02
                w = 0.55 * turn_sign
            else:
                error = self.wall_distance - side_range
                v = self.max_v * 0.55
                w = float(np.clip(turn_sign * 1.2 * error, -0.45, 0.45))
        self.publish_cmd(v, w)

    def choose_wall_side(self):
        left = self.range_in_sector(math.pi / 2.0, math.radians(25.0))
        right = self.range_in_sector(-math.pi / 2.0, math.radians(25.0))
        if left < right and left < self.wall_acquire_distance:
            return 'left'
        return 'right'

    def can_leave_wall(self, dist, angle_error):
        if self.wall_lock_count > 0:
            return False
        if abs(angle_error) > math.radians(35.0):
            return False
        if self.range_in_sector(0.0, math.radians(25.0)) < self.wall_leave_clearance:
            return False
        if self.range_in_sector(angle_error, math.radians(20.0)) < self.wall_leave_clearance:
            return False
        if self.bug_algorithm == 2:
            return dist < self.best_goal_distance - 0.08
        return True

    def range_in_sector(self, center, half_width):
        if self.scan is None:
            return math.inf

        best = math.inf
        angle = self.scan.angle_min
        for value in self.scan.ranges:
            if math.isfinite(value):
                delta = math.atan2(math.sin(angle - center), math.cos(angle - center))
                if abs(delta) <= half_width:
                    best = min(best, value)
            angle += self.scan.angle_increment
        return best

    def publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveNavigation()
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
