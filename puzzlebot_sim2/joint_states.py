import math
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class JointPublisher(Node):

    def __init__(self):
        super().__init__('joint_states')

        # Params
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_map_to_odom', 'false')

        #Read Params
        self.odom_frame = self.get_parameter('odom_frame').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.base_frame = self.get_parameter('base_frame').value

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.wr = 0.0
        self.wl = 0.0

        self.right_wheel_angle = 0.0
        self.left_wheel_angle = 0.0
        self.last_time = self.get_clock().now()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ['wheel_r_joint', 'wheel_l_joint']
        self.joint_state_msg.position = [0.0, 0.0]
        self.joint_state_msg.velocity = [0.0, 0.0]
        self.joint_state_msg.effort = [0.0, 0.0]

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(
            Float32,
            'wr',
            self.wr_callback,
            qos.qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32,
            'wl',
            self.wl_callback,
            qos.qos_profile_sensor_data,
        )

        self.publish_static_transforms()
        self.timer = self.create_timer(0.02, self.timer_callback)

    def publish_static_transforms(self):
        """Publish the fixed global frames used by the package."""
        now = self.get_clock().now().to_msg()

        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = now
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = self.odom_frame
        map_to_odom.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform([map_to_odom])

    def odom_callback(self, msg: Odometry):
        """Store the latest pose estimate from dead reckoning."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = 2.0 * math.atan2(
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

    def wr_callback(self, msg: Float32):
        """Store the latest right wheel angular speed."""
        self.wr = msg.data

    def wl_callback(self, msg: Float32):
        """Store the latest left wheel angular speed."""
        self.wl = msg.data

    def timer_callback(self):
        """Publish the dynamic base transform and wheel joint states."""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        self.right_wheel_angle += self.wr * dt
        self.left_wheel_angle += self.wl * dt

        base_transform = TransformStamped()
        base_transform.header.stamp = now.to_msg()
        base_transform.header.frame_id = self.odom_frame
        base_transform.child_frame_id = self.base_frame
        base_transform.transform.translation.x = self.x
        base_transform.transform.translation.y = self.y
        base_transform.transform.translation.z = 0.0
        base_transform.transform.rotation.z = math.sin(self.yaw / 2.0)
        base_transform.transform.rotation.w = math.cos(self.yaw / 2.0)

        self.joint_state_msg.header.stamp = now.to_msg()
        self.joint_state_msg.position = [
            self.right_wheel_angle,
            self.left_wheel_angle,
        ]
        self.joint_state_msg.velocity = [self.wr, self.wl]

        self.tf_broadcaster.sendTransform(base_transform)
        self.joint_state_pub.publish(self.joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()

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
