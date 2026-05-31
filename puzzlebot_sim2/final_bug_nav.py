#!/usr/bin/env python3
"""Final Challenge reactive Bug navigation for Puzzlebot.

Compatible with the Manchester puzzlebot_sim interface:
- subscribes: /scan (sensor_msgs/LaserScan), /odom (nav_msgs/Odometry)
- publishes:  /cmd_vel (geometry_msgs/Twist)
- optional: subscribes /goal_pose (geometry_msgs/Pose2D) for manual testing

The node can run an autonomous closed waypoint sequence with at least four goals.
It replaces the normal point controller because obstacle avoidance must directly
command /cmd_vel.
"""

import math
import signal
import sys
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class FinalBugNavigation(Node):
    def __init__(self):
        super().__init__('final_bug_navigation')

        # Algorithm / interface parameters
        self.declare_parameter('bug_mode', 0)  # 0 = Bug0-like, 2 = Bug2-like leave condition
        self.declare_parameter('bug_direction', 'fwcw')  # fwcw or fwccw
        self.declare_parameter('run_waypoint_sequence', True)
        self.declare_parameter('loop_closed_path', False)
        # Legacy / alternative parameter names kept for compatibility with configs
        self.declare_parameter('use_waypoints', False)
        self.declare_parameter('loop_waypoints', False)
        self.declare_parameter('goal_topic', 'goal_pose')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')

        # Four target points arranged to force obstacle circumnavigation.
        # Tune these to the final Gazebo world dimensions.
        # Declare both canonical and legacy waypoint parameter names.
        self.declare_parameter('waypoints_x', [1.20, 1.20, -0.80, -0.80, 0.00])
        self.declare_parameter('waypoints_y', [0.80, -0.80, -0.80, 0.80, 0.00])
        self.declare_parameter('waypoints_theta', [0.0, -1.57, 3.14, 1.57, 0.0])
        self.declare_parameter('waypoint_x', [])
        self.declare_parameter('waypoint_y', [])
        self.declare_parameter('waypoint_theta', [])

        # Controller parameters
        self.declare_parameter('timer_period', 0.05)  # 20 Hz control loop
        self.declare_parameter('goal_tolerance', 0.12)
        self.declare_parameter('max_linear_speed', 0.16)
        self.declare_parameter('max_angular_speed', 1.6)
        self.declare_parameter('heading_gain', 2.2)
        self.declare_parameter('distance_gain', 0.65)

        # Obstacle / wall-following parameters
        self.declare_parameter('front_clearance', 0.34)
        self.declare_parameter('wall_follow_safety', 0.30)
        self.declare_parameter('ahead_clearance_angle_deg', 35.0)
        self.declare_parameter('goal_heading_clear_angle_deg', 22.0)
        self.declare_parameter('line_distance_threshold_bug2', 0.15)
        self.declare_parameter('leave_goal_improvement', 0.03)
        self.declare_parameter('min_wall_follow_time', 1.0)

        self.bug_mode = int(self.get_parameter('bug_mode').value)
        self.bug_direction = str(self.get_parameter('bug_direction').value)
        # Support both new and legacy parameter names for enabling waypoints/looping.
        run_waypoint = bool(self.get_parameter('run_waypoint_sequence').value)
        legacy_use = bool(self.get_parameter('use_waypoints').value)
        self.run_waypoint_sequence = run_waypoint or legacy_use

        loop_way = bool(self.get_parameter('loop_closed_path').value)
        legacy_loop = bool(self.get_parameter('loop_waypoints').value)
        self.loop_closed_path = loop_way or legacy_loop

        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.timer_period = float(self.get_parameter('timer_period').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.heading_gain = float(self.get_parameter('heading_gain').value)
        self.distance_gain = float(self.get_parameter('distance_gain').value)

        self.front_clearance = float(self.get_parameter('front_clearance').value)
        self.wall_follow_safety = float(self.get_parameter('wall_follow_safety').value)
        self.ahead_clearance_angle = math.radians(
            float(self.get_parameter('ahead_clearance_angle_deg').value)
        )
        self.goal_heading_clear_angle = math.radians(
            float(self.get_parameter('goal_heading_clear_angle_deg').value)
        )
        self.line_distance_threshold_bug2 = float(
            self.get_parameter('line_distance_threshold_bug2').value
        )
        self.leave_goal_improvement = float(self.get_parameter('leave_goal_improvement').value)
        self.min_wall_follow_time = float(self.get_parameter('min_wall_follow_time').value)

        # Load waypoint arrays: prefer canonical names, fall back to legacy ones if present.
        default_waypoints_x = [1.20, 1.20, -0.80, -0.80, 0.00]
        default_waypoints_y = [0.80, -0.80, -0.80, 0.80, 0.00]

        xs = list(self.get_parameter('waypoints_x').value)
        ys = list(self.get_parameter('waypoints_y').value)
        ths = list(self.get_parameter('waypoints_theta').value)
        legacy_x = list(self.get_parameter('waypoint_x').value)
        legacy_y = list(self.get_parameter('waypoint_y').value)
        legacy_th = list(self.get_parameter('waypoint_theta').value)

        # Prefer explicit canonical parameters, but fall back to legacy parameters
        # if the old-style arrays are provided and the canonical arrays appear to be
        # still the default placeholder values.
        if legacy_x and legacy_y and (
            not xs or not ys or xs == default_waypoints_x and ys == default_waypoints_y
        ):
            xs = legacy_x
            ys = legacy_y
            ths = legacy_th if legacy_th else [0.0] * len(xs)
        self.waypoints = self.build_waypoints(xs, ys, ths)
        self.waypoint_index = 0

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.odom_received = False
        self.scan: Optional[LaserScan] = None
        self.pose_covariance = None

        # Goal state
        self.goal: Optional[Tuple[float, float, float]] = None
        self.goal_received = False
        self.active_goal_completed = False

        # Bug state
        self.state = 'go_to_goal'
        self.hit_point: Optional[Tuple[float, float]] = None
        self.line_start: Optional[Tuple[float, float]] = None
        self.min_goal_distance = math.inf
        self.wall_start_time = None

        # ROS interfaces
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(Pose2D, self.goal_topic, self.external_goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        signal.signal(signal.SIGINT, self.shutdown_function)

        if self.run_waypoint_sequence and self.waypoints:
            self.set_goal(self.waypoints[0], source='initial waypoint')

        self.get_logger().info(
            f'Final Bug Navigation ready | mode={self.bug_mode} direction={self.bug_direction} '
            f'waypoints={len(self.waypoints)} scan={self.scan_topic} odom={self.odom_topic} cmd={self.cmd_vel_topic}'
        )

    @staticmethod
    def build_waypoints(xs: List[float], ys: List[float], ths: List[float]) -> List[Tuple[float, float, float]]:
        n = min(len(xs), len(ys))
        if len(ths) < n:
            ths = ths + [0.0] * (n - len(ths))
        return [(float(xs[i]), float(ys[i]), float(ths[i])) for i in range(n)]

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def set_goal(self, goal: Tuple[float, float, float], source: str = 'goal'):
        self.goal = goal
        self.goal_received = True
        self.active_goal_completed = False
        self.state = 'go_to_goal'
        self.hit_point = None
        self.line_start = (self.x, self.y)
        self.min_goal_distance = math.inf
        self.wall_start_time = None
        self.get_logger().info(f'{source}: x={goal[0]:.2f}, y={goal[1]:.2f}, theta={math.degrees(goal[2]):.1f} deg')

    def external_goal_callback(self, msg: Pose2D):
        if self.run_waypoint_sequence:
            return
        self.set_goal((msg.x, msg.y, msg.theta), source='external goal')

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x = p.x
        self.y = p.y
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny, cosy)
        self.pose_covariance = msg.pose.covariance
        self.odom_received = True

    def scan_callback(self, msg: LaserScan):
        self.scan = msg

    def timer_callback(self):
        if not self.odom_received or self.scan is None:
            self.publish_stop()
            return

        if not self.goal_received or self.goal is None:
            self.publish_stop()
            return

        dist_to_goal, angle_to_goal = self.compute_goal_error()

        if dist_to_goal <= self.goal_tolerance:
            self.handle_goal_reached()
            return

        if self.state == 'go_to_goal':
            if self.path_blocked(angle_to_goal):
                self.state = 'follow_wall'
                self.hit_point = (self.x, self.y)
                self.line_start = (self.x, self.y) if self.line_start is None else self.line_start
                self.min_goal_distance = dist_to_goal
                self.wall_start_time = self.get_clock().now()
                self.get_logger().info('Obstacle detected -> follow_wall')
            else:
                v, w = self.go_to_goal_command(dist_to_goal, angle_to_goal)
                self.publish_velocity(v, w)
                return

        if self.state == 'follow_wall':
            current_line_distance = self.distance_to_m_line()
            self.min_goal_distance = min(self.min_goal_distance, dist_to_goal)

            if self.can_leave_wall(dist_to_goal, current_line_distance, angle_to_goal):
                self.state = 'go_to_goal'
                self.get_logger().info('Leave condition satisfied -> go_to_goal')
                v, w = self.go_to_goal_command(dist_to_goal, angle_to_goal)
            else:
                _, theta_closest = self.get_closest_object()
                v, w = self.follow_wall_command(theta_closest)

            self.publish_velocity(v, w)

    def handle_goal_reached(self):
        if not self.active_goal_completed:
            self.get_logger().info(f'Goal {self.waypoint_index + 1} reached')
            self.active_goal_completed = True

        self.publish_stop()

        if not self.run_waypoint_sequence:
            return

        next_index = self.waypoint_index + 1
        if next_index >= len(self.waypoints):
            if self.loop_closed_path:
                next_index = 0
            else:
                self.get_logger().info('Closed trajectory completed')
                self.goal_received = False
                return

        self.waypoint_index = next_index
        self.set_goal(self.waypoints[self.waypoint_index], source=f'waypoint {self.waypoint_index + 1}/{len(self.waypoints)}')

    def compute_goal_error(self) -> Tuple[float, float]:
        gx, gy, _ = self.goal
        dx = gx - self.x
        dy = gy - self.y
        dist = math.hypot(dx, dy)
        desired = math.atan2(dy, dx)
        return dist, self.normalize_angle(desired - self.theta)

    def go_to_goal_command(self, dist: float, heading_error: float) -> Tuple[float, float]:
        if abs(heading_error) > math.pi / 2.0:
            return 0.0, self.clamp(self.heading_gain * heading_error, self.max_angular_speed)

        v = min(self.distance_gain * dist, self.max_linear_speed)
        w = self.clamp(self.heading_gain * heading_error, self.max_angular_speed)
        if abs(heading_error) > 0.45:
            v *= 0.25
        return v, w

    def path_blocked(self, angle_to_goal: float) -> bool:
        return not self.is_sector_clear(angle_to_goal, self.ahead_clearance_angle, self.front_clearance)

    def goal_heading_clear(self, angle_to_goal: float) -> bool:
        return self.is_sector_clear(angle_to_goal, self.goal_heading_clear_angle, self.front_clearance)

    def is_sector_clear(self, center_angle: float, half_width: float, threshold: float) -> bool:
        min_range = math.inf
        angle = self.scan.angle_min
        for r in self.scan.ranges:
            if math.isfinite(r) and self.scan.range_min <= r <= self.scan.range_max:
                delta = self.normalize_angle(angle - center_angle)
                if abs(delta) <= half_width:
                    min_range = min(min_range, r)
            angle += self.scan.angle_increment
        return min_range > threshold

    def get_closest_object(self) -> Tuple[float, float]:
        best_range = math.inf
        best_angle = 0.0
        angle = self.scan.angle_min
        for r in self.scan.ranges:
            if math.isfinite(r) and self.scan.range_min <= r <= self.scan.range_max:
                if r < best_range:
                    best_range = r
                    best_angle = angle
            angle += self.scan.angle_increment
        return best_range, self.normalize_angle(best_angle)

    def follow_wall_command(self, theta_closest: float) -> Tuple[float, float]:
        # Obstacle avoidance direction: point away from closest object.
        theta_ao = self.normalize_angle(theta_closest + math.pi)

        # Follow-wall direction is perpendicular to the avoidance vector.
        if self.bug_direction == 'fwccw':
            theta_fw = self.normalize_angle(theta_ao + math.pi / 2.0)
        else:
            theta_fw = self.normalize_angle(theta_ao - math.pi / 2.0)

        v = 0.75 * self.max_linear_speed
        w = self.clamp(1.15 * theta_fw, self.max_angular_speed)
        return v, w

    def distance_to_m_line(self) -> float:
        if self.line_start is None or self.goal is None:
            return math.inf

        x0, y0 = self.line_start
        x1, y1, _ = self.goal
        x2, y2 = self.x, self.y
        dx = x1 - x0
        dy = y1 - y0

        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return math.hypot(x2 - x0, y2 - y0)

        return abs(dy * x2 - dx * y2 + x1 * y0 - y1 * x0) / math.hypot(dx, dy)

    def wall_follow_time_elapsed(self) -> bool:
        if self.wall_start_time is None:
            return True
        elapsed = (self.get_clock().now() - self.wall_start_time).nanoseconds * 1e-9
        return elapsed >= self.min_wall_follow_time

    def can_leave_wall(self, dist_to_goal: float, line_distance: float, angle_to_goal: float) -> bool:
        if not self.wall_follow_time_elapsed():
            return False
        if not self.goal_heading_clear(angle_to_goal):
            return False

        improved = dist_to_goal < (self.min_goal_distance - self.leave_goal_improvement)

        if self.bug_mode == 2:
            return improved and line_distance <= self.line_distance_threshold_bug2
        return improved

    def publish_velocity(self, v: float, w: float):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        self.publish_velocity(0.0, 0.0)

    def shutdown_function(self, signum, frame):
        self.get_logger().info('Shutdown requested -> stop robot')
        self.publish_stop()
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = FinalBugNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
