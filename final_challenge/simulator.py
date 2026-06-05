import math
import signal
import sys

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


def yaw_to_quat(yaw):
    return math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)


class PuzzlebotSimulator(Node):
    def __init__(self):
        super().__init__('puzzlebot_simulator')

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('theta0', 0.0)
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheel_base').value)
        self.x = float(self.get_parameter('x0').value)
        self.y = float(self.get_parameter('y0').value)
        self.theta = float(self.get_parameter('theta0').value)
        self.dt = 1.0 / float(self.get_parameter('update_rate').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.phi_r = 0.0
        self.phi_l = 0.0

        self.wr_pub = self.create_publisher(Float32, 'wr', 10)
        self.wl_pub = self.create_publisher(Float32, 'wl', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'sim_pose', 10)
        self.odom_pub = self.create_publisher(Odometry, 'sim_pose_odom', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)

        self.create_timer(self.dt, self.step)
        signal.signal(signal.SIGINT, self.shutdown_function)
        self.get_logger().info('Simulador listo: recibe /cmd_vel y publica /wr, /wl, /sim_pose.')

    def shutdown_function(self, signum, frame):
        self.get_logger().info('Shutting down simulator...')
        rclpy.shutdown()
        sys.exit(0)

    def cmd_callback(self, msg):
        self.v_cmd = float(msg.linear.x)
        self.w_cmd = float(msg.angular.z)

    def step(self):
        wr = (self.v_cmd + self.w_cmd * self.L / 2.0) / self.r
        wl = (self.v_cmd - self.w_cmd * self.L / 2.0) / self.r

        v = self.r * (wr + wl) / 2.0
        w = self.r * (wr - wl) / self.L

        self.x += v * math.cos(self.theta) * self.dt
        self.y += v * math.sin(self.theta) * self.dt
        self.theta += w * self.dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.phi_r += wr * self.dt
        self.phi_l += wl * self.dt

        now = self.get_clock().now().to_msg()
        q = yaw_to_quat(self.theta)

        msg_wr = Float32()
        msg_wr.data = float(wr)
        self.wr_pub.publish(msg_wr)

        msg_wl = Float32()
        msg_wl.data = float(wl)
        self.wl_pub.publish(msg_wl)

        joint = JointState()
        joint.header.stamp = now
        joint.name = ['wheel_r_joint', 'wheel_l_joint']
        joint.position = [float(self.phi_r), float(self.phi_l)]
        joint.velocity = [float(wr), float(wl)]
        self.joint_pub.publish(joint)

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = self.odom_frame
        pose.pose.position.x = float(self.x)
        pose.pose.position.y = float(self.y)
        pose.pose.orientation.w = float(q[0])
        pose.pose.orientation.x = float(q[1])
        pose.pose.orientation.y = float(q[2])
        pose.pose.orientation.z = float(q[3])
        self.pose_pub.publish(pose)

        odom = Odometry()
        odom.header = pose.header
        odom.child_frame_id = self.base_frame
        odom.pose.pose = pose.pose
        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.angular.z = float(w)
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotSimulator()
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
