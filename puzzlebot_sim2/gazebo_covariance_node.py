#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformException, TransformListener


class GazeboCovarianceNode(Node):
    def __init__(self):
        super().__init__("gazebo_covariance_node")

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("output_odom_topic", "/ekf_odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("marker_frame_prefix", "marker_")
        self.declare_parameter("marker_ids", [705, 706, 70, 703, 708, 75, 702])

        self.declare_parameter("growth_rate_xy", 0.0008)
        self.declare_parameter("growth_rate_theta", 0.0005)
        self.declare_parameter("marker_correction_factor", 0.35)
        self.declare_parameter("max_covariance_xy", 0.25)

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.output_odom_topic = str(self.get_parameter("output_odom_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.marker_frame_prefix = str(self.get_parameter("marker_frame_prefix").value)

        self.marker_ids = [int(v) for v in self.get_parameter("marker_ids").value]
        self.growth_rate_xy = float(self.get_parameter("growth_rate_xy").value)
        self.growth_rate_theta = float(self.get_parameter("growth_rate_theta").value)
        self.marker_correction_factor = float(
            self.get_parameter("marker_correction_factor").value
        )
        self.max_covariance_xy = float(self.get_parameter("max_covariance_xy").value)

        self.P = np.diag([0.02, 0.02, 0.03])
        self.last_time = self.get_clock().now()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_profile_sensor_data,
        )

        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 10)

        self.get_logger().info(
            f"Covariance node ready | input={self.odom_topic} output={self.output_odom_topic}"
        )

    def marker_visible(self):
        for marker_id in self.marker_ids:
            marker_frame = f"{self.marker_frame_prefix}{marker_id}"
            try:
                self.tf_buffer.lookup_transform(
                    self.base_frame,
                    marker_frame,
                    rclpy.time.Time(),
                )
                return True
            except TransformException:
                continue
        return False

    def odom_callback(self, msg: Odometry):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt < 0.0 or dt > 1.0:
            dt = 0.05

        speed = abs(msg.twist.twist.linear.x) + abs(msg.twist.twist.angular.z)

        self.P[0, 0] += self.growth_rate_xy * (1.0 + speed) * dt
        self.P[1, 1] += self.growth_rate_xy * (1.0 + speed) * dt
        self.P[2, 2] += self.growth_rate_theta * (1.0 + speed) * dt

        self.P[0, 0] = min(self.P[0, 0], self.max_covariance_xy)
        self.P[1, 1] = min(self.P[1, 1], self.max_covariance_xy)
        self.P[2, 2] = min(self.P[2, 2], 0.35)

        if self.marker_visible():
            self.P *= self.marker_correction_factor
            self.P[0, 0] = max(self.P[0, 0], 0.005)
            self.P[1, 1] = max(self.P[1, 1], 0.005)
            self.P[2, 2] = max(self.P[2, 2], 0.01)

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose = msg.pose.pose
        out.twist.twist = msg.twist.twist

        cov = [0.0] * 36
        cov[0] = float(self.P[0, 0])
        cov[1] = float(self.P[0, 1])
        cov[5] = float(self.P[0, 2])

        cov[6] = float(self.P[1, 0])
        cov[7] = float(self.P[1, 1])
        cov[11] = float(self.P[1, 2])

        cov[30] = float(self.P[2, 0])
        cov[31] = float(self.P[2, 1])
        cov[35] = float(self.P[2, 2])

        out.pose.covariance = cov
        out.twist.covariance = msg.twist.covariance

        self.odom_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboCovarianceNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
