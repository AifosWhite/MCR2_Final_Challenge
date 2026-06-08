#!/usr/bin/env python3
import math
import signal
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


DEFAULT_WAYPOINTS_X = [1.30, 2.82]
DEFAULT_WAYPOINTS_Y = [-0.30, -0.30]


class BugController(Node):

    STATE_GO_TO_GOAL  = "go_to_goal"
    STATE_FOLLOW_WALL = "follow_wall"
    STATE_STOP        = "stop"

    def __init__(self):
        super().__init__('bug_controller')

        self.declare_parameter('controller_update_rate',          25.0)
        self.declare_parameter('distance_tolerance',               0.12)
        self.declare_parameter('following_walls_distance',         0.25)
        self.declare_parameter('front_stop_distance',              0.30)
        self.declare_parameter('lookahead_distance',               0.30)
        self.declare_parameter('p2p_v_Kp',                        0.8)
        self.declare_parameter('p2p_w_Kp',                        1.2)
        self.declare_parameter('fw_w_Kp',                         1.0)
        self.declare_parameter('fw_e_Kp',                         3.5)
        self.declare_parameter('fw_linear_speed',                  0.12)
        self.declare_parameter('fw_outer_corner_angular_speed',    1.2)
        self.declare_parameter('fw_outer_corner_linear_speed',     0.16)
        self.declare_parameter('v_max',                            0.12)
        self.declare_parameter('w_max',                            1.2)
        self.declare_parameter('side_open_angle',                  0.5236)
        self.declare_parameter('front_open_angle',                 0.4)
        self.declare_parameter('controller_type',                  'BUG2')

        self.declare_parameter('lidar_yaw_offset',                 3.14159)
        self.declare_parameter('max_w_accel',                      4.0)
        self.declare_parameter('bug2_line_tol',                    0.15)
        self.declare_parameter('min_wall_follow_distance',         0.40)
        self.declare_parameter('waypoints_x',                      DEFAULT_WAYPOINTS_X)
        self.declare_parameter('waypoints_y',                      DEFAULT_WAYPOINTS_Y)
        self.declare_parameter('loop',                             True)
        self.declare_parameter('odom_topic',                       'odom')
        self.declare_parameter('scan_topic',                       'scan')
        self.declare_parameter('cmd_vel_topic',                    'cmd_vel')

        gp = self.get_parameter
        self.update_rate                   = float(gp('controller_update_rate').value)
        self.distance_tolerance            = float(gp('distance_tolerance').value)
        self.d_wall                        = float(gp('following_walls_distance').value)
        self.front_stop_distance           = float(gp('front_stop_distance').value)
        self.lookahead_distance            = float(gp('lookahead_distance').value)
        self.p2p_v_Kp                      = float(gp('p2p_v_Kp').value)
        self.p2p_w_Kp                      = float(gp('p2p_w_Kp').value)
        self.fw_w_Kp                       = float(gp('fw_w_Kp').value)
        self.fw_e_Kp                       = float(gp('fw_e_Kp').value)
        self.fw_linear_speed               = float(gp('fw_linear_speed').value)
        self.fw_outer_corner_angular_speed = float(gp('fw_outer_corner_angular_speed').value)
        self.fw_outer_corner_linear_speed  = float(gp('fw_outer_corner_linear_speed').value)
        self.v_max                         = float(gp('v_max').value)
        self.w_max                         = float(gp('w_max').value)
        self.side_open_angle               = float(gp('side_open_angle').value)
        self.front_open_angle              = float(gp('front_open_angle').value)
        self.controller_type               = str(gp('controller_type').value)
        self.lidar_yaw_offset              = float(gp('lidar_yaw_offset').value)
        self.max_w_accel                   = float(gp('max_w_accel').value)
        self.bug2_line_tol                 = float(gp('bug2_line_tol').value)
        self.min_wall_follow_distance      = float(gp('min_wall_follow_distance').value)
        self.loop                          = bool(gp('loop').value)
        odom_topic    = str(gp('odom_topic').value)
        scan_topic    = str(gp('scan_topic').value)
        cmd_vel_topic = str(gp('cmd_vel_topic').value)

        wx = list(gp('waypoints_x').value)
        wy = list(gp('waypoints_y').value)
        self.waypoints  = [(float(x), float(y)) for x, y in zip(wx, wy)]
        self.goal_index = 0

        self.create_subscription(Odometry,  odom_topic,  self.odom_callback,       qos_profile_sensor_data)
        self.create_subscription(LaserScan, scan_topic,  self.lidar_callback,      10)
        self.create_subscription(Pose2D,    'setpoint',  self.setpoint_callback,   qos_profile_sensor_data)
        self.cmd_vel_publisher      = self.create_publisher(Twist, cmd_vel_topic,  10)
        self.goal_reached_publisher = self.create_publisher(Bool,  'goal_reached', qos_profile_sensor_data)

        self.robot_pose           = Pose2D()
        self.goal_pose            = Pose2D()
        self.scan_ready           = False
        self.prev_w               = 0.0
        self.collision_time       = self.get_clock().now()
        self.min_front            = float('inf')
        self.min_side             = float('inf')
        self.min_back_side        = float('inf')
        self.min_back_side_out    = float('inf')
        self.closest_object_angle = 0.0
        self.lidar_min_range      = 0.15
        self.state                = self.STATE_GO_TO_GOAL
        self.fw_direction         = 'fwccw'
        self.d_gtg_at_hit         = float('inf')
        self.line_A = self.line_B = self.line_C = 0.0

        self._set_goal_from_list()
        self.get_logger().info(
            f'{self.controller_type} | {len(self.waypoints)} WPs | '
            f'loop={self.loop} | lidar_offset={self.lidar_yaw_offset:.4f} rad')

        signal.signal(signal.SIGINT, self.shutdown_function)
        self.create_timer(1.0 / self.update_rate, self.controller_callback)

    # ── Waypoints ────────────────────────────────────────────────────────────
    def _set_goal_from_list(self):
        gx, gy = self.waypoints[self.goal_index]
        self.goal_pose = Pose2D(x=gx, y=gy, theta=0.0)
        self.state = self.STATE_GO_TO_GOAL
        self._compute_start_line()
        self.get_logger().info(f'→ WP{self.goal_index}: ({gx:.2f}, {gy:.2f})')

    def _advance_waypoint(self):
        if self.goal_index + 1 >= len(self.waypoints):
            if self.loop:
                self.goal_index = 0
            else:
                self.get_logger().info('Ruta completa.')
                self.state = self.STATE_STOP
                return
        else:
            self.goal_index += 1
        self._set_goal_from_list()

    def setpoint_callback(self, msg):
        self.goal_pose = msg
        self._compute_start_line()
        self.state = self.STATE_GO_TO_GOAL

    # ── Callbacks ────────────────────────────────────────────────────────────
    def odom_callback(self, msg: Odometry):
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_pose.theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def lidar_callback(self, msg: LaserScan):
        self.scan_ready = True
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi

        ranges = np.array(msg.ranges)
        ranges = np.where(np.isfinite(ranges) & (ranges > msg.range_min), ranges, np.inf)

        self.closest_object_angle = norm_angle(
            msg.angle_min + int(np.argmin(ranges)) * msg.angle_increment
            - self.lidar_yaw_offset)
        self.lidar_min_range = msg.range_min

        eff_offset = msg.angle_min - self.lidar_yaw_offset
        inc        = msg.angle_increment
        rmin       = msg.range_min

        self.min_front = self._region_min(
            ranges, -msg.angle_min, eff_offset,
            self.front_open_angle, self.front_open_angle, inc, rmin)

        side_c = self._side_center(self.fw_direction, msg.angle_min)
        self.min_side = self._region_min(
            ranges, side_c, eff_offset,
            self.side_open_angle, self.side_open_angle, inc, rmin)
        self.min_back_side = self._region_min(
            ranges, side_c, eff_offset,
            self.side_open_angle * 0.5, self.side_open_angle, inc, rmin)
        self.min_back_side_out = self._region_min_outside(
            ranges, side_c, eff_offset,
            self.side_open_angle * 0.5, self.side_open_angle, inc, rmin)

    # ── Máquina de estados ───────────────────────────────────────────────────
    def controller_callback(self):
        if not self.scan_ready:
            return

        d_gtg     = self._dist_to_goal()
        theta_gtg = self._angle_to_goal()
        twist     = Twist()

        if self.state == self.STATE_STOP:
            self.cmd_vel_publisher.publish(Twist())
            return

        elif self.state == self.STATE_GO_TO_GOAL:
            if d_gtg < self.distance_tolerance:
                self.cmd_vel_publisher.publish(Twist())
                self.goal_reached_publisher.publish(Bool(data=True))
                self.get_logger().info(f'WP{self.goal_index} alcanzado.')
                self._advance_waypoint()
                return

            v, w = self._go_to_goal_control(d_gtg, theta_gtg)

            if self.min_front < self.front_stop_distance:
                self.get_logger().info(f'Hit point → {self.controller_type}')
                self.d_gtg_at_hit = d_gtg
                self.fw_direction = self._choose_fw_direction()
                if self.controller_type == 'BUG2':
                    self._compute_start_line()
                self.state = self.STATE_FOLLOW_WALL
                v, w = self._follow_wall_control()

        else:  # FOLLOW_WALL
            if (self.get_clock().now() - self.collision_time <
                    rclpy.duration.Duration(seconds=0.75)):
                v = -0.08
                w = (-1 if self.fw_direction == 'fwccw' else 1) * self.w_max
            else:
                v, w = self._follow_wall_control()
                if self.min_front < self.lidar_min_range + 0.01:
                    self.collision_time = self.get_clock().now()

            if self.controller_type == 'BUG0':
                if self._bug0_leave_condition(d_gtg, theta_gtg):
                    self.get_logger().info('Bug0: clear shot → go_to_goal')
                    self.state = self.STATE_GO_TO_GOAL
            elif self.controller_type == 'BUG2':
                if self._bug2_leave_condition(d_gtg):
                    self.get_logger().info('Bug2: m-line + progreso → go_to_goal')
                    self.state = self.STATE_GO_TO_GOAL

        v = float(np.clip(v, -self.v_max, self.v_max))
        w = float(np.clip(w, -self.w_max, self.w_max))
        if self.max_w_accel > 0.0:
            dw_max = self.max_w_accel / self.update_rate
            w = self.prev_w + float(np.clip(w - self.prev_w, -dw_max, dw_max))
        self.prev_w = w

        twist.linear.x  = v
        twist.angular.z = w
        self.cmd_vel_publisher.publish(twist)

        self.get_logger().info(
            f'[{self.state}] WP{self.goal_index} d={d_gtg:.2f} '
            f'front={self.min_front:.2f} side={self.min_side:.2f} '
            f'v={v:+.2f} w={w:+.2f}',
            throttle_duration_sec=1.0)

    # ── Controladores ────────────────────────────────────────────────────────
    def _go_to_goal_control(self, d_gtg, theta_gtg):
        e_theta = norm_angle(theta_gtg - self.robot_pose.theta)
        e_d     = d_gtg
        v = min(self.p2p_v_Kp * e_d, self.v_max)
        w = self.p2p_w_Kp * e_theta
        return v, w

    def _follow_wall_control(self):
        theta_ao = norm_angle(self.closest_object_angle + math.pi)
        theta_fw = norm_angle(theta_ao + (math.pi / 2 if self.fw_direction == 'fwccw' else -math.pi / 2))

        ed = (self.min_side - self.d_wall) if self.fw_direction == 'fwccw' else (self.d_wall - self.min_side)
        w  = self.fw_w_Kp * theta_fw + self.fw_e_Kp * ed
        v  = self.fw_linear_speed

        if self.min_front < 2 * self.front_stop_distance:
            v = self.fw_linear_speed / 2
        if self.min_front < self.front_stop_distance:
            v = 0.0
            w = (-1.0 if self.fw_direction == 'fwccw' else 1.0) * self.w_max * 0.8
        elif (self.min_back_side < self.lookahead_distance and
              self.min_back_side_out > self.lookahead_distance):
            v = self.fw_outer_corner_linear_speed
            w = (1.0 if self.fw_direction == 'fwccw' else -1.0) * self.fw_outer_corner_angular_speed

        return v, w

    # ── Condiciones de salida ────────────────────────────────────────────────
    def _bug0_leave_condition(self, d_gtg, theta_gtg):
        theta_ao   = norm_angle(self.closest_object_angle + math.pi)
        angle_diff = abs(norm_angle(theta_ao - theta_gtg))
        progress   = d_gtg < (self.d_gtg_at_hit - self.distance_tolerance)
        clear_shot = angle_diff < math.pi / 2
        return progress and clear_shot

    def _bug2_leave_condition(self, d_gtg):
        on_line  = self._distance_to_start_line() < self.bug2_line_tol
        progress = d_gtg < (self.d_gtg_at_hit - self.min_wall_follow_distance)
        front_ok = self.min_front > self.front_stop_distance + 0.05
        return on_line and progress and front_ok

    # ── Helpers ──────────────────────────────────────────────────────────────
    def _dist_to_goal(self):
        return math.hypot(self.goal_pose.x - self.robot_pose.x,
                          self.goal_pose.y - self.robot_pose.y)

    def _angle_to_goal(self):
        return math.atan2(self.goal_pose.y - self.robot_pose.y,
                          self.goal_pose.x - self.robot_pose.x)

    def _choose_fw_direction(self):
        theta_ao   = norm_angle(self.closest_object_angle + math.pi)
        theta_fwc  = norm_angle(theta_ao - math.pi / 2)
        theta_fwcc = norm_angle(theta_ao + math.pi / 2)
        direction  = 'fwcw' if abs(theta_fwc) <= abs(theta_fwcc) else 'fwccw'
        self.get_logger().info(f'fw_direction: {direction}')
        return direction

    def _compute_start_line(self):
        dx = self.goal_pose.x - self.robot_pose.x
        dy = self.goal_pose.y - self.robot_pose.y
        if abs(dx) < 1e-6:
            self.line_A, self.line_B, self.line_C = 1.0, 0.0, -self.robot_pose.x
        else:
            m = dy / dx
            self.line_A, self.line_B, self.line_C = m, -1.0, self.robot_pose.y - m * self.robot_pose.x

    def _distance_to_start_line(self):
        num = abs(self.line_A * self.robot_pose.x + self.line_B * self.robot_pose.y + self.line_C)
        den = math.sqrt(self.line_A ** 2 + self.line_B ** 2)
        return num / den if den > 1e-9 else float('inf')

    def _side_center(self, direction, angle_min):
        return (math.pi / 2 if direction == 'fwccw' else 3 * math.pi / 2) - angle_min

    @staticmethod
    def _norm_shift(a):
        a = math.atan2(math.sin(a), math.cos(a))
        return a if a >= 0 else 2.0 * math.pi + a

    def _region_min(self, r, center, offset, front_open, back_open, inc, rmin):
        if center < math.pi:
            a0 = self._norm_shift(center - offset - front_open)
            a1 = self._norm_shift(center - offset + back_open)
        else:
            a0 = self._norm_shift(center - offset - back_open)
            a1 = self._norm_shift(center - offset + front_open)
        return self._min_idx(r, int(a0 / inc), int(a1 / inc), rmin)

    def _region_min_outside(self, r, center, offset, front_open, back_open, inc, rmin):
        a0 = self._norm_shift(center - offset - front_open)
        a1 = self._norm_shift(center - offset + back_open)
        return self._min_idx_outside(r, int(a0 / inc), int(a1 / inc), rmin)

    @staticmethod
    def _min_idx(r, i0, i1, rmin):
        vals = np.concatenate((r[i0:], r[:i1])) if i0 > i1 else r[i0:i1]
        return float('inf') if vals.size == 0 else max(float(np.min(vals)), rmin)

    @staticmethod
    def _min_idx_outside(r, i0, i1, rmin):
        vals = r[i1:i0] if i0 > i1 else np.concatenate((r[i1:], r[:i0]))
        return float('inf') if vals.size == 0 else max(float(np.min(vals)), rmin)

    def shutdown_function(self, signum, frame):
        self.get_logger().info('Shutting down. Stopping robot...')
        self.cmd_vel_publisher.publish(Twist())
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = BugController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_vel_publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()