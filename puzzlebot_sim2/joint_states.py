import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math
import numpy as np

class JointStatesNode(Node):
    def __init__(self):
        super().__init__('joint_states')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('wheel_radius', 0.05)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wr = 0.0
        self.wl = 0.0
        self.right_angle = 0.0
        self.left_angle = 0.0

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Float32, '/wr', self.wr_callback, 10)
        self.create_subscription(Float32, '/wl', self.wl_callback, 10)

        self.publish_static_tf()
        self.timer = self.create_timer(0.02, self.timer_callback)

    def publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # theta from quaternion
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny, cosy)

    def wr_callback(self, msg):
        self.wr = msg.data

    def wl_callback(self, msg):
        self.wl = msg.data

    def timer_callback(self):
        dt = 0.02
        self.right_angle += self.wr * dt
        self.left_angle += self.wl * dt

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['wheel_right_joint', 'wheel_left_joint']
        js.position = [self.right_angle, self.left_angle]
        self.joint_pub.publish(js)

        t = TransformStamped()
        t.header.stamp = js.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
