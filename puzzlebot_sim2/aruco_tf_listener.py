#!/usr/bin/env python3
"""Utility ROS2 node to inspect ArUco marker transforms.

This node is intentionally small: it listens to a TF transform between the robot
base and a marker frame and prints distance/bearing. It is useful for validating
that aruco_ros is publishing marker frames before enabling EKF correction.
"""

import math

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class ArucoTfListener(Node):
    def __init__(self):
        super().__init__('aruco_tf_listener')

        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'marker_0')
        self.declare_parameter('timer_period', 0.2)

        self.parent_frame = str(self.get_parameter('parent_frame').value)
        self.child_frame = str(self.get_parameter('child_frame').value)
        timer_period = float(self.get_parameter('timer_period').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'Listening for TF {self.parent_frame} -> {self.child_frame}'
        )

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                rclpy.time.Time(),
            )
        except TransformException as ex:
            self.get_logger().debug(
                f'Could not transform {self.child_frame} to {self.parent_frame}: {ex}'
            )
            return

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        distance = math.hypot(x, y)
        bearing = math.atan2(y, x)

        self.get_logger().info(
            f'{self.child_frame}: x={x:.3f} m, y={y:.3f} m, '
            f'distance={distance:.3f} m, bearing={bearing:.3f} rad'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTfListener()
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
