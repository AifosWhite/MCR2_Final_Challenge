import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped


class WaypointDriver(Node):
    """Simple waypoint follower that reads `pose_sim` and publishes `cmd_vel`."""

    def __init__(self):
        super().__init__('waypoint_driver')

        # Parameters
        self.declare_parameter('waypoints', ['1.5,-0.5', '2.0,0.0', '2.0,2.0'])
        self.declare_parameter('tolerance', 0.15)
        self.declare_parameter('linear_kp', 0.8)
        self.declare_parameter('angular_kp', 1.5)

        raw_wps = self.get_parameter('waypoints').value
        # Accept several formats: list of [x,y], list of strings 'x,y', or YAML list
        self.waypoints = []
        try:
            if isinstance(raw_wps, list):
                for item in raw_wps:
                    if isinstance(item, str):
                        parts = item.split(',')
                        self.waypoints.append([float(parts[0]), float(parts[1])])
                    elif isinstance(item, (list, tuple)):
                        self.waypoints.append([float(item[0]), float(item[1])])
            elif isinstance(raw_wps, str):
                # single string like "[[1.5, -0.5], [2.0, 0.0]]"
                import ast
                parsed = ast.literal_eval(raw_wps)
                for item in parsed:
                    self.waypoints.append([float(item[0]), float(item[1])])
        except Exception:
            self.get_logger().warning('Could not parse waypoints parameter; using empty list')
        self.tolerance = float(self.get_parameter('tolerance').value)
        self.lk = float(self.get_parameter('linear_kp').value)
        self.ak = float(self.get_parameter('angular_kp').value)

        # State
        self.current_wp = 0
        self.pose = None

        # ROS interfaces
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(PoseStamped, 'pose_sim', self.pose_callback, 10)

        # Control loop
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Waypoint driver ready. Waypoints: %s' % str(self.waypoints))

    def pose_callback(self, msg: PoseStamped):
        self.pose = msg

    def control_loop(self):
        if self.pose is None or not self.waypoints:
            return

        wx, wy = self.waypoints[self.current_wp]
        px = self.pose.pose.position.x
        py = self.pose.pose.position.y

        dx = wx - px
        dy = wy - py
        dist = math.hypot(dx, dy)

        # Extract yaw from quaternion (assuming planar z,w only)
        yaw = 2.0 * math.atan2(self.pose.pose.orientation.z, self.pose.pose.orientation.w)
        desired = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))

        cmd = Twist()

        if dist > self.tolerance:
            cmd.linear.x = min(self.lk * dist, 0.5)
            cmd.angular.z = max(min(self.ak * yaw_err, 1.5), -1.5)
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            # advance waypoint
            prev = self.current_wp
            self.current_wp = (self.current_wp + 1) % len(self.waypoints)
            self.get_logger().info(f'Reached waypoint {prev} -> next {self.current_wp}: {self.waypoints[self.current_wp]}')

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
