import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import math

class CovarianceMarker(Node):
    def __init__(self):
        super().__init__('covariance_marker')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/covariance_ellipse', 1)
        self.latest_odom = None
        self.timer = self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg):
        self.latest_odom = msg

    def timer_callback(self):
        if self.latest_odom is None:
            return
        msg = self.latest_odom
        cov = msg.pose.covariance
        Pxx = cov[0]
        Pyy = cov[7]
        Pxy = cov[1]
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Ellipse axes
        a = 2.0 * math.sqrt(max(Pxx, 0.0))
        b = 2.0 * math.sqrt(max(Pyy, 0.0))

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = msg.header.stamp
        marker.ns = 'covariance'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.05
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = a
        marker.scale.y = b
        marker.scale.z = 0.05
        marker.color.r = 0.2
        marker.color.g = 0.6
        marker.color.b = 1.0
        marker.color.a = 0.45
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = CovarianceMarker()
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
