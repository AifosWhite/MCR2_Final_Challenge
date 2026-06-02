#!/usr/bin/env python3
"""
bug_controller.py  -  Control de navegacion Bug0/Bug2 con seguimiento de pared.

Adaptado del controlador del equipo "servo_sirvo_fisico" (que funciona en el
Puzzlebot fisico) al paquete puzzlebot_sim2. Cambios respecto al original:
  - Sin dependencia de transforms3d: el yaw se saca con atan2 directo del
    quaternion (igual que tu localisation.py).
  - math_utils integrado en este archivo (no requiere modulo utils/).
  - Recorre una LISTA de waypoints (closed loop), publicando internamente el
    siguiente setpoint. El original esperaba un setpoint externo por topico;
    aqui se mantiene ese modo (topico 'setpoint') Y se agrega el modo lista.

Caracteristicas que tu nodo anterior NO tenia y por las que este es mejor en
fisico:
  - Recuperacion de colision (retrocede y gira si toca pared).
  - Control de esquina exterior con lookahead (no pierde la pared en esquinas).
  - Ajuste del rango angular del LiDAR fisico (overwrite_min_max_angles).
  - Orientacion final al llegar al punto (tolerancia angular).
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


# ---- math utils (integrados) ----------------------------------------------
def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def shift_0_2pi(t):
    return t if t >= 0 else 2.0 * math.pi + t


def norm_and_shift(t):
    return shift_0_2pi(norm_angle(t))


def angle_between_poses(p_from, p_to):
    a = math.atan2(p_to.y - p_from.y, p_to.x - p_from.x)
    return norm_angle(a - p_from.theta)


def euclidean(p1, p2):
    return math.hypot(p1.x - p2.x, p1.y - p2.y)


# Ruta segura por defecto (origin-centered, frame odom/EKF). Closed loop por
# el centro de los pasillos del maze de simulacion, validada wall-safe.
DEFAULT_WAYPOINTS_X = [1.20, 1.00, 0.80, 0.05, 0.10, 1.30, 1.00, 0.85, 0.25,
                       0.65, 0.50, 0.35, 0.15, -0.25, -0.45, -1.30, -0.40,
                       -0.20, 0.00, 0.10, 0.90, 1.05, 1.20]
DEFAULT_WAYPOINTS_Y = [-1.00, -1.30, -1.10, -0.40, -0.20, 1.00, 0.70, 0.75,
                       1.30, 0.90, 0.20, 0.05, 0.15, 0.55, 0.50, -0.30, 0.55,
                       0.50, -0.05, -0.45, -1.20, -1.15, -1.00]


class BugController(Node):
    def __init__(self):
        super().__init__('bug_controller')

        # ---- Parametros ---------------------------------------------------
        self.declare_parameter('controller_update_rate', 25.0)
        self.declare_parameter('distance_tolerance', 0.12)
        self.declare_parameter('angular_tolerance', 0.20)
        self.declare_parameter('angular_adjustment', 0.1745)
        self.declare_parameter('following_walls_distance', 0.25)
        self.declare_parameter('front_stop_distance', 0.30)
        self.declare_parameter('lookahead_distance', 0.30)
        self.declare_parameter('p2p_v_Kp', 0.8)
        self.declare_parameter('p2p_w_Kp', 1.2)
        self.declare_parameter('fw_w_Kp', 1.2)
        self.declare_parameter('fw_e_Kp', 8.0)
        self.declare_parameter('fw_linear_speed', 0.10)
        self.declare_parameter('fw_outer_corner_angular_speed', 2.0)
        self.declare_parameter('fw_outer_corner_linear_speed', 0.15)
        self.declare_parameter('v_max', 0.12)
        self.declare_parameter('w_max', 1.5)
        self.declare_parameter('side_open_angle', 0.5236)
        self.declare_parameter('front_open_angle', 0.4)
        self.declare_parameter('target_open_angle', 0.5)
        self.declare_parameter('controller_type', 'BUG2')
        self.declare_parameter('overwrite_min_max_angles', False)  # True en fisico
        # Angulo (rad) del scan que apunta al FRENTE real del robot. Si el rplidar
        # esta montado girado, las regiones front/side/target quedan desalineadas
        # (p.ej. lee la pared de atras como "frente"). Calibra: 0 si el 0 del scan
        # mira al frente; ~3.14159 si mira hacia atras.
        self.declare_parameter('lidar_yaw_offset', 3.14159)
        # Limite de aceleracion angular (rad/s^2). Suaviza los giros evitando que
        # w salte de golpe (sobre todo en following_walls). 0 = sin limite.
        self.declare_parameter('max_w_accel', 4.0)
        # Tolerancia (m) para re-enganchar la m-line y salir de wall-following (BUG2).
        self.declare_parameter('bug2_line_tol', 0.15)

        # Modo lista de waypoints (si vacio, espera setpoints por topico).
        self.declare_parameter('waypoints_x', DEFAULT_WAYPOINTS_X)
        self.declare_parameter('waypoints_y', DEFAULT_WAYPOINTS_Y)
        self.declare_parameter('loop', True)
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')

        gp = self.get_parameter
        self.update_rate = float(gp('controller_update_rate').value)
        self.distance_tolerance = float(gp('distance_tolerance').value)
        self.angular_tolerance = float(gp('angular_tolerance').value)
        self.angular_adjustment = float(gp('angular_adjustment').value)
        self.following_walls_distance = float(gp('following_walls_distance').value)
        self.front_stop_distance = float(gp('front_stop_distance').value)
        self.lookahead_distance = float(gp('lookahead_distance').value)
        self.p2p_v_Kp = float(gp('p2p_v_Kp').value)
        self.p2p_w_Kp = float(gp('p2p_w_Kp').value)
        self.fw_w_Kp = float(gp('fw_w_Kp').value)
        self.fw_e_Kp = float(gp('fw_e_Kp').value)
        self.fw_linear_speed = float(gp('fw_linear_speed').value)
        self.fw_outer_corner_angular_speed = float(gp('fw_outer_corner_angular_speed').value)
        self.fw_outer_corner_linear_speed = float(gp('fw_outer_corner_linear_speed').value)
        self.v_max = float(gp('v_max').value)
        self.w_max = float(gp('w_max').value)
        self.side_open_angle = float(gp('side_open_angle').value)
        self.front_open_angle = float(gp('front_open_angle').value)
        self.target_open_angle = float(gp('target_open_angle').value)
        self.controller_type = str(gp('controller_type').value)
        self.using_real_robot = bool(gp('overwrite_min_max_angles').value)
        self.lidar_yaw_offset = float(gp('lidar_yaw_offset').value)
        self.max_w_accel = float(gp('max_w_accel').value)
        self.bug2_line_tol = float(gp('bug2_line_tol').value)
        self.loop = bool(gp('loop').value)
        odom_topic = str(gp('odom_topic').value)
        scan_topic = str(gp('scan_topic').value)
        cmd_vel_topic = str(gp('cmd_vel_topic').value)

        wx = list(gp('waypoints_x').value)
        wy = list(gp('waypoints_y').value)
        self.waypoints = [(float(x), float(y)) for x, y in zip(wx, wy)]
        self.use_waypoint_list = len(self.waypoints) > 0
        self.goal_index = 0

        # ---- Subs / pubs --------------------------------------------------
        self.create_subscription(Pose2D, 'setpoint', self.setpoint_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, odom_topic, self.odom_callback,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, scan_topic, self.lidar_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.goal_reached_publisher = self.create_publisher(
            Bool, 'goal_reached', qos_profile_sensor_data)

        # ---- Estado -------------------------------------------------------
        self.robot_pose = Pose2D()
        self.robot_setpoint = Pose2D()
        self.closest_object_angle = 0.0
        self.controller_mode = 'p2p_controller'
        self.start_pose = None
        self.target_line = None
        self.lidar_min_range = 0.15
        self.collision_time = self.get_clock().now()
        self.prev_w = 0.0   # ultimo w publicado (para limitar aceleracion angular)

        self.fw_dirs = {'left': 1, 'right': -1}
        self.fw_dir = 'right'
        self.min_side = float('inf')
        self.min_front = float('inf')
        self.min_back_side = float('inf')
        self.min_back_side_out = float('inf')
        self.scan_ready = False

        self.controller_timer = self.create_timer(
            1.0 / self.update_rate, self.controller_callback)

        if self.use_waypoint_list:
            self._set_goal_from_list()
            self.get_logger().info(
                f'Bug{self.controller_type[-1]} con {len(self.waypoints)} '
                f'waypoints, loop={self.loop}.')
        else:
            self.get_logger().info(
                'Bug controller en modo setpoint externo (topico "setpoint").')

    # ---- Lista de waypoints ----------------------------------------------
    def _set_goal_from_list(self):
        gx, gy = self.waypoints[self.goal_index]
        self.robot_setpoint = Pose2D(x=gx, y=gy, theta=self.robot_pose.theta)
        self.controller_mode = 'p2p_controller'
        self.start_pose = None

    def _advance_waypoint(self):
        if self.goal_index + 1 >= len(self.waypoints):
            if self.loop:
                self.goal_index = 0
            else:
                self.get_logger().info('Ruta completa.')
                return False
        else:
            self.goal_index += 1
        self._set_goal_from_list()
        self.get_logger().info(
            f'Siguiente WP{self.goal_index}: '
            f'({self.robot_setpoint.x:.2f}, {self.robot_setpoint.y:.2f}).')
        return True

    # ---- Callbacks --------------------------------------------------------
    def setpoint_callback(self, msg):
        if self.use_waypoint_list:
            return  # en modo lista ignoramos setpoints externos
        self.robot_setpoint = msg

    def odom_callback(self, msg):
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_pose.theta = math.atan2(siny, cosy)

    def lidar_callback(self, msg):
        self.scan_ready = True
        if self.using_real_robot:
            msg.angle_min = 0.0
            msg.angle_max = 2.0 * math.pi
        ranges = np.array(msg.ranges)
        # Angulo del obstaculo mas cercano, en frame del ROBOT (resta el offset de montaje).
        self.closest_object_angle = norm_angle(
            msg.angle_min + np.argmin(ranges) * msg.angle_increment
            - self.lidar_yaw_offset)
        self.lidar_min_range = msg.range_min

        # eff_offset alinea TODAS las regiones (front/side/target) con el frente real:
        # el centro efectivo de cada region es (center - eff_offset) = angulo_robot + lidar_yaw_offset.
        eff_offset = msg.angle_min - self.lidar_yaw_offset

        theta_err = angle_between_poses(self.robot_pose, self.robot_setpoint)
        min_target = self._region_min(ranges, theta_err, eff_offset,
                                      self.target_open_angle,
                                      self.target_open_angle,
                                      msg.angle_increment, msg.range_min)

        side_c = self._side_center(self.fw_dir, msg.angle_min)
        self.min_side = self._region_min(ranges, side_c, eff_offset,
                                         self.side_open_angle,
                                         self.side_open_angle,
                                         msg.angle_increment, msg.range_min)
        front_c = -msg.angle_min
        self.min_front = self._region_min(ranges, front_c, eff_offset,
                                          self.front_open_angle,
                                          self.front_open_angle,
                                          msg.angle_increment, msg.range_min)
        self.min_back_side = self._region_min(ranges, side_c, eff_offset,
                                              self.side_open_angle * 0.5,
                                              self.side_open_angle,
                                              msg.angle_increment, msg.range_min)
        self.min_back_side_out = self._region_min_outside(
            ranges, side_c, eff_offset, self.side_open_angle * 0.5,
            self.side_open_angle, msg.angle_increment, msg.range_min)

        # Maquina de estados de modo
        if self.controller_mode == 'p2p_controller':
            if self.min_front < self.following_walls_distance * 1.5:
                self.fw_dir = np.random.choice(list(self.fw_dirs.keys()))
                m = (self.robot_setpoint.y - self.robot_pose.y) / \
                    ((self.robot_setpoint.x - self.robot_pose.x) + 1e-6)
                b = self.robot_pose.y - m * self.robot_pose.x
                self.target_line = lambda x: m * x + b
                self.controller_mode = 'following_walls'
        elif self.controller_mode == 'following_walls':
            if self.controller_type == 'BUG0':
                if min_target > euclidean(self.robot_pose, self.robot_setpoint):
                    self.controller_mode = 'p2p_controller'
            elif self.controller_type == 'BUG2':
                err_y = self.target_line(self.robot_pose.x) - self.robot_pose.y
                # Tolerancia de re-enganche a la m-line aflojada (antes 0.05): con
                # 5 cm casi nunca salia del wall-following y se quedaba dando vueltas.
                if abs(err_y) < self.bug2_line_tol and \
                        min_target > self.front_stop_distance + 0.2:
                    self.controller_mode = 'p2p_controller'
                    self.start_pose = None

    # ---- Control ----------------------------------------------------------
    def controller_callback(self):
        if not self.scan_ready:
            return

        d = euclidean(self.robot_pose, self.robot_setpoint)
        twist = Twist()

        if self.controller_mode == 'following_walls':
            if self.get_clock().now() - self.collision_time < \
                    rclpy.duration.Duration(seconds=0.75):
                v = -0.08
                w = -self.fw_dirs[self.fw_dir] * self.w_max
            else:
                sep = norm_angle(self.closest_object_angle + math.pi)
                tangent = norm_angle(sep + self.fw_dirs[self.fw_dir] * math.pi / 2)
                v = self.fw_linear_speed
                if self.min_front < self.front_stop_distance:
                    v = 0.04
                elif self.min_front < 2 * self.front_stop_distance:
                    v /= 2
                dist_err = self.min_side - self.following_walls_distance
                w = self.fw_w_Kp * tangent + \
                    self.fw_dirs[self.fw_dir] * self.fw_e_Kp * dist_err
                if self.min_back_side < self.lookahead_distance and \
                        self.min_back_side_out > self.lookahead_distance:
                    v = self.fw_outer_corner_linear_speed
                    w = self.fw_dirs[self.fw_dir] * self.fw_outer_corner_angular_speed
                if self.min_front < self.lidar_min_range + 0.01:
                    self.collision_time = self.get_clock().now()

            v = float(np.clip(v, -self.v_max, self.v_max))
            w = float(np.clip(w, -self.w_max, self.w_max))
            twist.linear.x = v
            twist.angular.z = w

        elif self.controller_mode == 'p2p_controller':
            theta_err = angle_between_poses(self.robot_pose, self.robot_setpoint)
            v = self.p2p_v_Kp * d
            w = self.p2p_w_Kp * theta_err
            if self.min_front < self.front_stop_distance:
                self.start_pose = Pose2D(x=self.robot_pose.x,
                                         y=self.robot_pose.y,
                                         theta=self.robot_pose.theta)
                self.controller_mode = 'following_walls'
                self.collision_time = self.get_clock().now()
                return
            if abs(theta_err) > self.angular_adjustment:
                v = 0.0
            if d < self.distance_tolerance:
                v = 0.0
                w = self.p2p_w_Kp * norm_angle(
                    self.robot_setpoint.theta - self.robot_pose.theta)
            v = float(np.clip(v, -self.v_max, self.v_max))
            w = float(np.clip(w, -self.w_max, self.w_max))
            twist.linear.x = v
            twist.angular.z = w

        # Llegada al waypoint
        reached = d < self.distance_tolerance
        if self.use_waypoint_list:
            # en modo lista no exigimos orientacion final salvo ultimo punto
            if reached:
                self.prev_w = 0.0
                self.cmd_vel_publisher.publish(Twist())
                self._advance_waypoint()
                return
        else:
            if reached and abs(norm_angle(
                    self.robot_setpoint.theta - self.robot_pose.theta)) \
                    < self.angular_tolerance:
                self.goal_reached_publisher.publish(Bool(data=True))
                self.cmd_vel_publisher.publish(Twist())
                self.get_logger().info('Objetivo alcanzado.')
                return

        # Suaviza: limita la aceleracion angular para que w no salte de golpe
        # (sobre todo en following_walls / backup de colision) -> giros suaves.
        if self.max_w_accel > 0.0:
            dw_max = self.max_w_accel / self.update_rate
            twist.angular.z = self.prev_w + float(
                np.clip(twist.angular.z - self.prev_w, -dw_max, dw_max))
        self.prev_w = twist.angular.z

        # Log de estado para tunear navegacion (1 Hz). Modo, distancia a meta,
        # holguras del LiDAR y el comando publicado.
        self.get_logger().info(
            f'[{self.controller_mode}] WP{self.goal_index} d={d:.2f} '
            f'front={self.min_front:.2f} side={self.min_side:.2f} '
            f'v={twist.linear.x:+.2f} w={twist.angular.z:+.2f}',
            throttle_duration_sec=1.0)

        self.cmd_vel_publisher.publish(twist)

    # ---- Helpers de regiones LiDAR ---------------------------------------
    def _indices(self, a0, a1, inc):
        a0 = norm_and_shift(a0)
        a1 = norm_and_shift(a1)
        return int(a0 / inc), int(a1 / inc)

    def _min_idx(self, r, i0, i1, rmin):
        if i0 > i1:
            vals = np.concatenate((r[i0:], r[:i1]))
        else:
            vals = r[i0:i1]
        if vals.size == 0:
            return float('inf')
        return max(float(np.min(vals)), rmin)

    def _min_idx_outside(self, r, i0, i1, rmin):
        if i0 > i1:
            vals = r[i1:i0]
        else:
            vals = np.concatenate((r[i1:], r[:i0]))
        if vals.size == 0:
            return float('inf')
        return max(float(np.min(vals)), rmin)

    def _region_min(self, r, center, offset, front_open, back_open, inc, rmin):
        if center < math.pi:
            a0 = norm_and_shift(center - offset - front_open)
            a1 = norm_and_shift(center - offset + back_open)
        else:
            a0 = norm_and_shift(center - offset - back_open)
            a1 = norm_and_shift(center - offset + front_open)
        i0, i1 = self._indices(a0, a1, inc)
        return self._min_idx(r, i0, i1, rmin)

    def _region_min_outside(self, r, center, offset, front_open, back_open, inc, rmin):
        a0 = norm_and_shift(center - offset - front_open)
        a1 = norm_and_shift(center - offset + back_open)
        i0, i1 = self._indices(a0, a1, inc)
        return self._min_idx_outside(r, i0, i1, rmin)

    def _side_center(self, direction, angle_min):
        if direction == 'left':
            return math.pi / 2 - angle_min
        return 3 * math.pi / 2 - angle_min


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
