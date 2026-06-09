#!/usr/bin/env python3
"""Wheel joint-state publisher for the physical Puzzlebot.

This node replaces the Gazebo JointStatePublisher plugin when running on hardware.
It only publishes /joint_states. It does not publish map->odom or odom->base_footprint.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


class PuzzlebotJointStatePublisher(Node):
    def __init__(self):
        super().__init__('puzzlebot_joint_state_publisher')

        self.declare_parameter('right_wheel_topic', 'wr')
        self.declare_parameter('left_wheel_topic', 'wl')
        self.declare_parameter('right_wheel_joint', 'wheel_r_joint')
        self.declare_parameter('left_wheel_joint', 'wheel_l_joint')
        self.declare_parameter('rate_hz', 30.0)

        self.right_topic = str(self.get_parameter('right_wheel_topic').value)
        self.left_topic = str(self.get_parameter('left_wheel_topic').value)
        self.right_joint = str(self.get_parameter('right_wheel_joint').value)
        self.left_joint = str(self.get_parameter('left_wheel_joint').value)
        rate_hz = max(1.0, float(self.get_parameter('rate_hz').value))

        self.wr = 0.0
        self.wl = 0.0
        self.right_angle = 0.0
        self.left_angle = 0.0
        self.last_time = self.get_clock().now()

        self.create_subscription(Float32, self.right_topic, self.wr_callback, qos_profile_sensor_data)
        self.create_subscription(Float32, self.left_topic, self.wl_callback, qos_profile_sensor_data)
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)

        self.get_logger().info(
            f'Joint state publisher ready. Inputs: {self.right_topic}, {self.left_topic}; joints: {self.right_joint}, {self.left_joint}'
        )

    def wr_callback(self, msg: Float32):
        self.wr = float(msg.data)

    def wl_callback(self, msg: Float32):
        self.wl = float(msg.data)

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 1.0:
            return

        self.right_angle += self.wr * dt
        self.left_angle += self.wl * dt

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = [self.right_joint, self.left_joint]
        msg.position = [self.right_angle, self.left_angle]
        msg.velocity = [self.wr, self.wl]
        msg.effort = []
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotJointStatePublisher()
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