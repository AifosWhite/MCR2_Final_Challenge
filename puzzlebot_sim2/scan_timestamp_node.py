import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanTimestampNode(Node):
    def __init__(self):
        super().__init__('scan_timestamp_node')
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.create_subscription(LaserScan, '/scan_gz', self.scan_callback, 10)

    def scan_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanTimestampNode()
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
