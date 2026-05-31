import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def yaw_to_quat(yaw):
    return math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)


class Localisation(Node):
    def __init__(self):
        super().__init__('localisation')

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('theta0', 0.0)
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('kr', 0.002)
        self.declare_parameter('kl', 0.002)
        self.declare_parameter('use_ekf', True)
        self.declare_parameter('ekf_r_dist', 0.05)
        self.declare_parameter('ekf_r_bearing', 0.05)
        self.declare_parameter('marker_ids', [70, 706, 75, 701, 703, 705, 708, 702])
        self.declare_parameter('marker_pos_x', [-1.403, -0.433, 0.792, 1.143, 0.792, -0.109, -0.373, 0.266])
        self.declare_parameter('marker_pos_y', [0.224, 0.854, 1.218, 0.230, -0.354, -1.067, -0.370, -1.311])

        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheel_base').value)
        self.dt = 1.0 / float(self.get_parameter('update_rate').value)
        self.x = float(self.get_parameter('x0').value)
        self.y = float(self.get_parameter('y0').value)
        self.theta = float(self.get_parameter('theta0').value)
        self.world_frame = str(self.get_parameter('world_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.kr = float(self.get_parameter('kr').value)
        self.kl = float(self.get_parameter('kl').value)
        self.use_ekf = bool(self.get_parameter('use_ekf').value)
        self.ekf_r_dist = float(self.get_parameter('ekf_r_dist').value)
        self.ekf_r_bearing = float(self.get_parameter('ekf_r_bearing').value)

        ids = list(self.get_parameter('marker_ids').value)
        xs = list(self.get_parameter('marker_pos_x').value)
        ys = list(self.get_parameter('marker_pos_y').value)
        self.markers = {int(i): (float(x), float(y)) for i, x, y in zip(ids, xs, ys)}

        self.wr = 0.0
        self.wl = 0.0
        self.sigma = np.zeros((3, 3))
        self.latest_detection = None

        self.create_subscription(Float32, 'wr', self.wr_callback, 10)
        self.create_subscription(Float32, 'wl', self.wl_callback, 10)
        self.create_subscription(Float32MultiArray, '/aruco/detections', self.aruco_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.publish_static_tf()
        self.create_timer(self.dt, self.step)
        self.get_logger().info('Localisation lista: dead reckoning con correccion ArUco opcional.')

    def publish_static_tf(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.world_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(tf)

    def wr_callback(self, msg):
        self.wr = float(msg.data)

    def wl_callback(self, msg):
        self.wl = float(msg.data)

    def aruco_callback(self, msg):
        if len(msg.data) >= 3:
            self.latest_detection = int(msg.data[0]), float(msg.data[1]), float(msg.data[2])

    def step(self):
        v = self.r * (self.wr + self.wl) / 2.0
        w = self.r * (self.wr - self.wl) / self.L
        c = math.cos(self.theta)
        s = math.sin(self.theta)

        h = np.array([
            [1.0, 0.0, -v * self.dt * s],
            [0.0, 1.0, v * self.dt * c],
            [0.0, 0.0, 1.0],
        ])
        grad = 0.5 * self.r * self.dt * np.array([
            [c, c],
            [s, s],
            [2.0 / self.L, -2.0 / self.L],
        ])
        wheel_noise = np.diag([self.kr * abs(self.wr), self.kl * abs(self.wl)])
        self.sigma = h @ self.sigma @ h.T + grad @ wheel_noise @ grad.T

        self.x += v * c * self.dt
        self.y += v * s * self.dt
        self.theta += w * self.dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        if self.use_ekf and self.latest_detection is not None:
            marker_id, dist, bearing = self.latest_detection
            self.latest_detection = None
            self.ekf_correct(marker_id, dist, bearing)

        self.publish_odom(v, w)

    def ekf_correct(self, marker_id, z_dist, z_bearing):
        if marker_id not in self.markers:
            return

        mx, my = self.markers[marker_id]
        dx = mx - self.x
        dy = my - self.y
        p = dx * dx + dy * dy
        if p < 1e-8:
            return

        expected_dist = math.sqrt(p)
        expected_bearing = math.atan2(dy, dx) - self.theta
        expected_bearing = math.atan2(math.sin(expected_bearing), math.cos(expected_bearing))

        innovation = np.array([
            z_dist - expected_dist,
            math.atan2(math.sin(z_bearing - expected_bearing), math.cos(z_bearing - expected_bearing)),
        ])
        g = np.array([
            [-dx / expected_dist, -dy / expected_dist, 0.0],
            [dy / p, -dx / p, -1.0],
        ])
        r = np.diag([self.ekf_r_dist ** 2, self.ekf_r_bearing ** 2])
        z = g @ self.sigma @ g.T + r
        k = self.sigma @ g.T @ np.linalg.inv(z)
        delta = k @ innovation

        self.x += float(delta[0])
        self.y += float(delta[1])
        self.theta += float(delta[2])
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.sigma = (np.eye(3) - k @ g) @ self.sigma
        self.sigma = 0.5 * (self.sigma + self.sigma.T)

    def publish_odom(self, v, w):
        now = self.get_clock().now().to_msg()
        q = yaw_to_quat(self.theta)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.orientation.w = float(q[0])
        odom.pose.pose.orientation.x = float(q[1])
        odom.pose.pose.orientation.y = float(q[2])
        odom.pose.pose.orientation.z = float(q[3])
        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.angular.z = float(w)
        odom.pose.covariance = self.pack_covariance()
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = float(self.x)
        tf.transform.translation.y = float(self.y)
        tf.transform.rotation.w = float(q[0])
        tf.transform.rotation.x = float(q[1])
        tf.transform.rotation.y = float(q[2])
        tf.transform.rotation.z = float(q[3])
        self.tf_broadcaster.sendTransform(tf)

    def pack_covariance(self):
        cov = [0.0] * 36
        cov[0] = float(self.sigma[0, 0])
        cov[1] = float(self.sigma[0, 1])
        cov[5] = float(self.sigma[0, 2])
        cov[6] = float(self.sigma[1, 0])
        cov[7] = float(self.sigma[1, 1])
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
