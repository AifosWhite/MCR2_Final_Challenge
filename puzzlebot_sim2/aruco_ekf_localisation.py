#!/usr/bin/env python3
"""Experimental ArUco-assisted EKF localisation for the final challenge.

Prediction uses differential-drive wheel speeds (/wr, /wl). Correction uses TF
measurements from aruco_ros marker frames, converted to a range-bearing
measurement (distance, angle) in the robot base frame. The map position of each
marker is configured in YAML.

The node is designed to be safe for development: if no marker TF is available,
it only performs odometry prediction and covariance growth.
"""

import math
from typing import Dict, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener


class ArucoEkfLocalisation(Node):
    def __init__(self):
        super().__init__('aruco_ekf_localisation')

        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('theta0', 0.0)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('kr', 0.002)
        self.declare_parameter('kl', 0.002)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('output_topic', 'ekf_odom')
        self.declare_parameter('timer_period', 0.05)

        # Marker map definition. Lists must have the same length.
        self.declare_parameter('marker_ids', [0, 1, 2, 3])
        self.declare_parameter('marker_x', [1.0, 1.0, -1.0, -1.0])
        self.declare_parameter('marker_y', [1.0, -1.0, -1.0, 1.0])
        self.declare_parameter('marker_frame_prefix', 'marker_')
        # Modelo de medicion rango-azimut: ruido (desv. estandar) por componente.
        self.declare_parameter('measurement_noise_range', 0.05)
        self.declare_parameter('measurement_noise_bearing', 0.02)

        self.x = float(self.get_parameter('x0').value)
        self.y = float(self.get_parameter('y0').value)
        self.theta = float(self.get_parameter('theta0').value)
        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheel_base').value)
        self.kr = float(self.get_parameter('kr').value)
        self.kl = float(self.get_parameter('kl').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        timer_period = float(self.get_parameter('timer_period').value)
        self.marker_frame_prefix = str(self.get_parameter('marker_frame_prefix').value)
        self.measurement_noise_range = float(self.get_parameter('measurement_noise_range').value)
        self.measurement_noise_bearing = float(self.get_parameter('measurement_noise_bearing').value)

        ids = [int(v) for v in self.get_parameter('marker_ids').value]
        xs = [float(v) for v in self.get_parameter('marker_x').value]
        ys = [float(v) for v in self.get_parameter('marker_y').value]
        self.markers: Dict[int, Tuple[float, float]] = {
            ids[i]: (xs[i], ys[i]) for i in range(min(len(ids), len(xs), len(ys)))
        }

        self.P = np.diag([0.02, 0.02, 0.03])
        self.wr = 0.0
        self.wl = 0.0
        self.last_time = self.get_clock().now()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Float32, 'wr', self.wr_callback, qos.qos_profile_sensor_data)
        self.create_subscription(Float32, 'wl', self.wl_callback, qos.qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, self.output_topic, 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'Aruco EKF ready | markers={list(self.markers.keys())} output={self.output_topic}'
        )

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def wr_callback(self, msg: Float32):
        self.wr = msg.data

    def wl_callback(self, msg: Float32):
        self.wl = msg.data

    def predict(self, dt: float):
        dr = self.r * self.wr * dt
        dl = self.r * self.wl * dt
        dc = 0.5 * (dr + dl)
        dtheta = (dr - dl) / self.L
        theta_mid = self.theta + 0.5 * dtheta

        self.x += dc * math.cos(theta_mid)
        self.y += dc * math.sin(theta_mid)
        self.theta = self.normalize_angle(self.theta + dtheta)

        # H_k: Jacobiano del modelo de movimiento (presentacion: H = Jacobiano de movimiento)
        H = np.array([
            [1.0, 0.0, -dc * math.sin(theta_mid)],
            [0.0, 1.0,  dc * math.cos(theta_mid)],
            [0.0, 0.0,  1.0],
        ])

        # V: Jacobiano respecto al ruido de las ruedas (mapea el ruido de ruedas a Q_k)
        V = np.array([
            [0.5 * math.cos(theta_mid) - dc * math.sin(theta_mid) / (2.0 * self.L),
             0.5 * math.cos(theta_mid) + dc * math.sin(theta_mid) / (2.0 * self.L)],
            [0.5 * math.sin(theta_mid) + dc * math.cos(theta_mid) / (2.0 * self.L),
             0.5 * math.sin(theta_mid) - dc * math.cos(theta_mid) / (2.0 * self.L)],
            [1.0 / self.L, -1.0 / self.L],
        ])

        wheel_noise = np.diag([self.kr * abs(dr), self.kl * abs(dl)])
        Q = V @ wheel_noise @ V.T
        # Sigma_k = H_k Sigma_{k-1} H_k^T + Q_k
        self.P = H @ self.P @ H.T + Q
        self.P = 0.5 * (self.P + self.P.T)

    def correct_with_marker(self, marker_id: int, marker_position: Tuple[float, float]):
        marker_frame = f'{self.marker_frame_prefix}{marker_id}'
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                marker_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return

        # Medicion real z_k = [rango, azimut], obtenida del marcador visto en el
        # frame del robot (modelo de medicion rango-azimut de la presentacion).
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        z = np.array([
            math.hypot(tx, ty),
            math.atan2(ty, tx),
        ])

        # Medicion esperada zhat_k = g(mu_hat_k, m_i)
        #   g1 = sqrt((mx - sx)^2 + (my - sy)^2)
        #   g2 = atan2(my - sy, mx - sx) - s_theta
        mx, my = marker_position
        dx = mx - self.x
        dy = my - self.y
        p = dx * dx + dy * dy
        if p < 1e-9:
            return
        sqrt_p = math.sqrt(p)

        z_hat = np.array([
            sqrt_p,
            self.normalize_angle(math.atan2(dy, dx) - self.theta),
        ])

        # G_k: Jacobiano del modelo de medicion (presentacion: G = Jacobiano de medicion)
        G = np.array([
            [-dx / sqrt_p, -dy / sqrt_p,  0.0],
            [ dy / p,      -dx / p,      -1.0],
        ])

        # R_k: ruido del sensor (rango, azimut)
        R = np.diag([
            self.measurement_noise_range ** 2,
            self.measurement_noise_bearing ** 2,
        ])

        # Innovacion y_k = z_k - zhat_k (componente angular envuelta a [-pi, pi])
        innovation = z - z_hat
        innovation[1] = self.normalize_angle(innovation[1])

        # Z_k = G_k Sigma_hat_k G_k^T + R_k ;  K_k = Sigma_hat_k G_k^T Z_k^-1
        Z = G @ self.P @ G.T + R
        K = self.P @ G.T @ np.linalg.inv(Z)
        correction = K @ innovation

        # mu_k = mu_hat_k + K_k y_k
        self.x += correction[0]
        self.y += correction[1]
        self.theta = self.normalize_angle(self.theta + correction[2])

        # Sigma_k = (I - K_k G_k) Sigma_hat_k
        I = np.eye(3)
        self.P = (I - K @ G) @ self.P
        self.P = 0.5 * (self.P + self.P.T)

    def publish_odom(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = self.P[0, 0]
        msg.pose.covariance[1] = self.P[0, 1]
        msg.pose.covariance[5] = self.P[0, 2]
        msg.pose.covariance[6] = self.P[1, 0]
        msg.pose.covariance[7] = self.P[1, 1]
        msg.pose.covariance[11] = self.P[1, 2]
        msg.pose.covariance[30] = self.P[2, 0]
        msg.pose.covariance[31] = self.P[2, 1]
        msg.pose.covariance[35] = self.P[2, 2]
        self.odom_pub.publish(msg)

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        self.predict(dt)
        for marker_id, marker_position in self.markers.items():
            self.correct_with_marker(marker_id, marker_position)
        self.publish_odom(now)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoEkfLocalisation()
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
