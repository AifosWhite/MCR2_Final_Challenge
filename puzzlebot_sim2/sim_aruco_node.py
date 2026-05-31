#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class SimArucoNode(Node):
    def __init__(self):
        super().__init__('sim_aruco_node')

        self.declare_parameter('pose_topic', '/pose_sim')
        self.declare_parameter('detections_topic', '/aruco/detections')
        self.declare_parameter('marker_ids', [70,    706,    75,    701,    703,    705,    708,    702])
        self.declare_parameter('marker_pos_x', [-1.400, -0.430,  0.868,  1.140,  0.872, -0.110, -0.408,  0.231])
        self.declare_parameter('marker_pos_y', [ 0.240,  0.866,  1.220,  0.242, -0.350, -1.055, -0.370, -1.310])

        pose_topic = str(self.get_parameter('pose_topic').value)
        detections_topic = str(self.get_parameter('detections_topic').value)

        ids = [int(v) for v in self.get_parameter('marker_ids').value]
        xs = [float(v) for v in self.get_parameter('marker_pos_x').value]
        ys = [float(v) for v in self.get_parameter('marker_pos_y').value]
        self.markers = list(zip(ids, xs, ys))

        self.pose: Optional[Tuple[float, float, float]] = None
        self.max_range = 2.0
        self.fov = math.radians(85.0)

        self.pub = self.create_publisher(Float32MultiArray, detections_topic, 10)
        self.create_subscription(PoseStamped, pose_topic, self.pose_callback, 10)
        self.timer = self.create_timer(0.10, self.timer_callback)

        self.get_logger().info(f'Sim ArUco ready with {len(self.markers)} markers')

    @staticmethod
    def wrap(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def pose_callback(self, msg: PoseStamped):
        q = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.pose = (msg.pose.position.x, msg.pose.position.y, yaw)

    def timer_callback(self):
        if self.pose is None:
            return

        x, y, yaw = self.pose
        visible = []

        for marker_id, mx, my in self.markers:
            dx = mx - x
            dy = my - y
            distance = math.hypot(dx, dy)
            bearing = self.wrap(math.atan2(dy, dx) - yaw)

            if distance <= self.max_range and abs(bearing) <= self.fov / 2.0:
                visible.append((distance, marker_id, bearing))

        if not visible:
            return

        distance, marker_id, bearing = min(visible, key=lambda item: item[0])
        msg = Float32MultiArray()
        msg.data = [float(marker_id), float(distance), float(bearing)]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimArucoNode()
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
