#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class DistPrinter(Node):
    def __init__(self, target_id=None):
        super().__init__('aruco_dist_printer')
        self.target_id = float(target_id) if target_id is not None else None
        self.sub = self.create_subscription(Float32MultiArray, '/aruco/detections', self.cb, 10)

    def cb(self, msg: Float32MultiArray):
        data = msg.data
        if not data:
            return
        # Expect single detection as [id, distance, bearing]
        try:
            idv = int(data[0])
            dist = float(data[1]) if len(data) > 1 else None
            bearing = float(data[2]) if len(data) > 2 else None
        except Exception:
            return
        if self.target_id is None or idv == self.target_id:
            print(f'ID={idv} distance={dist:.3f} bearing={bearing:.3f}')


def main():
    target = int(sys.argv[1]) if len(sys.argv) > 1 else None
    rclpy.init()
    node = DistPrinter(target)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
