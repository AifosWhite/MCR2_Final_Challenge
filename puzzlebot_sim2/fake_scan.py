"""Publish a simple fake LaserScan so navigation can run in simulation.

This publishes fixed large ranges so the robot perceives no obstacles
and can follow waypoints in the simple simulator.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class FakeScan(Node):
    def __init__(self):
        super().__init__('fake_scan')

        # Parameters
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('num_readings', 180)
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('range_max', 10.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.rate = float(self.get_parameter('rate').value)
        self.num_readings = int(self.get_parameter('num_readings').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        self.pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        self.get_logger().info(f'Publishing fake LaserScan on "{self.scan_topic}" @ {self.rate}Hz')

    def timer_callback(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'

        msg.angle_min = -math.pi / 2.0
        msg.angle_max = math.pi / 2.0
        msg.angle_increment = (msg.angle_max - msg.angle_min) / max(1, self.num_readings - 1)
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.rate
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        # Large constant ranges; navigation will see a clear path.
        msg.ranges = [self.range_max] * self.num_readings
        msg.intensities = [0.0] * self.num_readings

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeScan()
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
