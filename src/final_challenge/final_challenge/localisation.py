import math
import signal
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker


def yaw_to_quat(yaw):
    return math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)


class Localisation(Node):
    def __init__(self):
        super().__init__('localisation')

        self.declare_parameter('wheel_radius',          0.05)
        self.declare_parameter('wheel_base',            0.19)
        self.declare_parameter('update_rate',           50.0)
        self.declare_parameter('x0',                    0.0)
        self.declare_parameter('y0',                    0.0)
        self.declare_parameter('theta0',                0.0)
        self.declare_parameter('world_frame',           'map')
        self.declare_parameter('odom_frame',            'odom')
        self.declare_parameter('base_frame',            'base_footprint')
        self.declare_parameter('kr',                    0.002)
        self.declare_parameter('kl',                    0.002)
        self.declare_parameter('use_ekf',               True)
        self.declare_parameter('ekf_r_dist',            0.05)
        self.declare_parameter('ekf_r_bearing',         0.05)
        self.declare_parameter('use_ground_truth_pose', False)
        self.declare_parameter('marker_ids',    [70, 75, 701, 702, 703, 705, 706, 708])
        self.declare_parameter('marker_pos_x',  [1.85, 2.75, 2.82, 0.27, 1.24, 0.89, 2.455, 1.185])
        self.declare_parameter('marker_pos_y',  [-0.30, -2.40, 0.00, -1.83, -2.07, -1.20, -1.255, -1.21])

        gp = self.get_parameter
        self.r       = float(gp('wheel_radius').value)
        self.L       = float(gp('wheel_base').value)
        self.dt      = 1.0 / float(gp('update_rate').value)
        self.x       = float(gp('x0').value)
        self.y       = float(gp('y0').value)
        self.theta   = float(gp('theta0').value)
        self.world_frame = str(gp('world_frame').value)
        self.odom_frame  = str(gp('odom_frame').value)
        self.base_frame  = str(gp('base_frame').value)
        self.kr          = float(gp('kr').value)
        self.kl          = float(gp('kl').value)
        self.use_ekf     = bool(gp('use_ekf').value)
        self.ekf_r_dist    = float(gp('ekf_r_dist').value)
        self.ekf_r_bearing = float(gp('ekf_r_bearing').value)

        ground_truth_value = gp('use_ground_truth_pose').value
        if isinstance(ground_truth_value, str):
            self.use_ground_truth_pose = ground_truth_value.lower() in ('1', 'true', 'yes')
        else:
            self.use_ground_truth_pose = bool(ground_truth_value)

        ids = list(gp('marker_ids').value)
        xs  = list(gp('marker_pos_x').value)
        ys  = list(gp('marker_pos_y').value)
        self.markers = {int(i): (float(x), float(y)) for i, x, y in zip(ids, xs, ys)}

        self.wr               = 0.0
        self.wl               = 0.0
        self.sigma            = np.zeros((3, 3))
        self.latest_detection = None
        self.latest_ground_truth = None
        self.prev_ns          = self.get_clock().now().nanoseconds

        # Suscriptores — solo lo que el EKF necesita (sin /scan)
        self.create_subscription(Float32,           'wr',                self.wr_callback,           qos_profile_sensor_data)
        self.create_subscription(Float32,           'wl',                self.wl_callback,           qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/aruco/detections', self.aruco_callback,         10)
        self.create_subscription(Odometry,          '/ground_truth',     self.ground_truth_callback,  10)

        # Publicadores
        self.odom_pub        = self.create_publisher(Odometry, 'odom',               10)
        self.cov_marker_pub  = self.create_publisher(Marker,   'covariance_ellipse', 10)
        self.tf_broadcaster  = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.publish_static_tf()
        self.create_timer(self.dt, self.step)
        signal.signal(signal.SIGINT, self.shutdown_function)
        self.get_logger().info('Localisation lista.')

    # ── Callbacks ────────────────────────────────────────────────────────────
    def wr_callback(self, msg):  self.wr = float(msg.data)
    def wl_callback(self, msg):  self.wl = float(msg.data)

    def aruco_callback(self, msg):
        if len(msg.data) >= 3:
            self.latest_detection = int(msg.data[0]), float(msg.data[1]), float(msg.data[2])

    def ground_truth_callback(self, msg):
        q = msg.pose.pose.orientation
        siny  = 2.0 * (q.w * q.z + q.x * q.y)
        cosy  = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.latest_ground_truth = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            math.atan2(siny, cosy),
        )

    def shutdown_function(self, signum, frame):
        self.get_logger().info('Shutting down localisation...')
        rclpy.shutdown()
        sys.exit(0)

    # ── TF estático map → odom ───────────────────────────────────────────────
    def publish_static_tf(self):
        tf = TransformStamped()
        tf.header.stamp    = self.get_clock().now().to_msg()
        tf.header.frame_id = self.world_frame
        tf.child_frame_id  = self.odom_frame
        tf.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(tf)

    # ── Step EKF (50 Hz) ─────────────────────────────────────────────────────
    def step(self):
        now_ns = self.get_clock().now().nanoseconds
        dt     = max((now_ns - self.prev_ns) * 1e-9, 1e-6)
        self.prev_ns = now_ns

        v = self.r * (self.wr + self.wl) / 2.0
        w = self.r * (self.wr - self.wl) / self.L
        c = math.cos(self.theta)
        s = math.sin(self.theta)

        # Propagación de covarianza
        h = np.array([
            [1.0, 0.0, -v * dt * s],
            [0.0, 1.0,  v * dt * c],
            [0.0, 0.0,  1.0],
        ])
        grad = 0.5 * self.r * dt * np.array([
            [c,  c],
            [s,  s],
            [2.0 / self.L, -2.0 / self.L],
        ])
        wheel_noise = np.diag([self.kr * abs(self.wr), self.kl * abs(self.wl)])
        self.sigma  = h @ self.sigma @ h.T + grad @ wheel_noise @ grad.T

        # Covarianza mínima
        self.sigma[0, 0] = max(self.sigma[0, 0], 0.002)
        self.sigma[1, 1] = max(self.sigma[1, 1], 0.002)
        self.sigma[2, 2] = max(self.sigma[2, 2], 0.001)

        # Dead reckoning
        self.x     += v * c * dt
        self.y     += v * s * dt
        self.theta += w * dt
        self.theta  = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Ground truth override
        if self.use_ground_truth_pose and self.latest_ground_truth is not None:
            self.x, self.y, self.theta = self.latest_ground_truth

        # Corrección EKF con ArUco
        if self.use_ekf and self.latest_detection is not None:
            marker_id, dist, bearing = self.latest_detection
            self.latest_detection = None
            self.ekf_correct(marker_id, dist, bearing)

        self.publish_odom(v, w)

    # ── Corrección EKF ───────────────────────────────────────────────────────
    def ekf_correct(self, marker_id, z_dist, z_bearing):
        if marker_id not in self.markers:
            return

        mx, my = self.markers[marker_id]
        dx = mx - self.x
        dy = my - self.y
        p  = dx * dx + dy * dy
        if p < 1e-8:
            return

        expected_dist    = math.sqrt(p)
        expected_bearing = math.atan2(math.sin(math.atan2(dy, dx) - self.theta),
                                      math.cos(math.atan2(dy, dx) - self.theta))

        innovation = np.array([
            z_dist - expected_dist,
            math.atan2(math.sin(z_bearing - expected_bearing),
                       math.cos(z_bearing - expected_bearing)),
        ])

        if abs(innovation[0]) > 0.5 or abs(innovation[1]) > 0.45:
            self.get_logger().warn(
                f'ArUco {marker_id} rechazado: '
                f'dist={innovation[0]:+.3f} bearing={innovation[1]:+.3f}',
                throttle_duration_sec=0.5)
            return

        g = np.array([
            [-dx / expected_dist, -dy / expected_dist, 0.0],
            [dy / p,              -dx / p,             -1.0],
        ])
        R  = np.diag([self.ekf_r_dist ** 2, self.ekf_r_bearing ** 2])
        Z  = g @ self.sigma @ g.T + R
        K  = self.sigma @ g.T @ np.linalg.inv(Z)
        delta = K @ innovation

        self.x     += float(delta[0])
        self.y     += float(delta[1])
        self.theta += float(delta[2])
        self.theta  = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.sigma  = (np.eye(3) - K @ g) @ self.sigma
        self.sigma  = 0.5 * (self.sigma + self.sigma.T)

        self.get_logger().info(
            f'ArUco {marker_id} | dist_real={z_dist:.3f} dist_est={expected_dist:.3f} '
            f'err={z_dist - expected_dist:+.3f} | '
            f'pose=({self.x:.3f}, {self.y:.3f}, {math.degrees(self.theta):.1f} deg) '
            f'corr=({float(delta[0]):+.3f}, {float(delta[1]):+.3f})',
            throttle_duration_sec=0.5)

    # ── Publicar odometría + TF + elipse de covarianza ───────────────────────
    def publish_odom(self, v, w):
        now = self.get_clock().now().to_msg()
        q   = yaw_to_quat(self.theta)

        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame
        odom.pose.pose.position.x    = float(self.x)
        odom.pose.pose.position.y    = float(self.y)
        odom.pose.pose.orientation.w = float(q[0])
        odom.pose.pose.orientation.x = float(q[1])
        odom.pose.pose.orientation.y = float(q[2])
        odom.pose.pose.orientation.z = float(q[3])
        odom.twist.twist.linear.x    = float(v)
        odom.twist.twist.angular.z   = float(w)
        odom.pose.covariance         = self.pack_covariance()
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header           = odom.header
        tf.child_frame_id   = self.base_frame
        tf.transform.translation.x = float(self.x)
        tf.transform.translation.y = float(self.y)
        tf.transform.rotation.w    = float(q[0])
        tf.transform.rotation.x    = float(q[1])
        tf.transform.rotation.y    = float(q[2])
        tf.transform.rotation.z    = float(q[3])
        self.tf_broadcaster.sendTransform(tf)

        self.publish_covariance_ellipse(now)

    def publish_covariance_ellipse(self, stamp):
        cov_2d = self.sigma[:2, :2]
        eigenvalues, eigenvectors = np.linalg.eigh(cov_2d)
        scale_x = 2.0 * math.sqrt(max(float(eigenvalues[0]), 1e-9))
        scale_y = 2.0 * math.sqrt(max(float(eigenvalues[1]), 1e-9))
        angle   = math.atan2(float(eigenvectors[1, 1]), float(eigenvectors[0, 1]))

        m = Marker()
        m.header.stamp    = stamp
        m.header.frame_id = self.odom_frame
        m.ns      = 'covariance'
        m.id      = 0
        m.type    = Marker.CYLINDER
        m.action  = Marker.ADD
        m.pose.position.x    = float(self.x)
        m.pose.position.y    = float(self.y)
        m.pose.position.z    = 0.0
        m.pose.orientation.w = math.cos(angle / 2.0)
        m.pose.orientation.z = math.sin(angle / 2.0)
        m.scale.x = max(scale_x, 0.01)
        m.scale.y = max(scale_y, 0.01)
        m.scale.z = 0.01
        m.color.a = 0.45
        m.color.r = 0.55
        m.color.b = 0.85
        self.cov_marker_pub.publish(m)

    def pack_covariance(self):
        cov = [0.0] * 36
        cov[0]  = float(self.sigma[0, 0])
        cov[1]  = float(self.sigma[0, 1])
        cov[5]  = float(self.sigma[0, 2])
        cov[6]  = float(self.sigma[1, 0])
        cov[7]  = float(self.sigma[1, 1])
        cov[11] = float(self.sigma[1, 2])
        cov[30] = float(self.sigma[2, 0])
        cov[31] = float(self.sigma[2, 1])
        cov[35] = float(self.sigma[2, 2])
        return cov


def main(args=None):
    rclpy.init(args=args)
    node = Localisation()
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