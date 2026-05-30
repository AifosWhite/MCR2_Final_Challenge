"""Part 1: differential-drive simulator for the Puzzlebot."""

import math
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Float32


class PuzzlebotSim(Node):

    def __init__(self):
        super().__init__('puzzlebot_sim')

        # Params
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('theta0', 0.0)

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('odom_frame', 'odom')

        # Read Params
        self.x = self.get_parameter('x0').value
        self.y = self.get_parameter('y0').value
        self.theta = self.get_parameter('theta0').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.odom_frame = self.get_parameter('odom_frame').value

        # Latest commanded body velocities.
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.last_time = self.get_clock().now()

        # Read commanded velocities from teleop or test publishers.
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Publish wheel speeds and scalar pose signals for plotting.
        self.wr_pub = self.create_publisher(Float32, 'wr', 10)
        self.wl_pub = self.create_publisher(Float32, 'wl', 10)
        self.x_pub = self.create_publisher(Float32, 'sim_x', 10)
        self.y_pub = self.create_publisher(Float32, 'sim_y', 10)
        self.theta_pub = self.create_publisher(Float32, 'sim_theta', 10)

        # Publish the full robot pose for visualization/TF consumers.
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_sim', 10)

        # Run the simulator at 50 Hz.
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info('puzzlebot_sim ->/cmd_vel')

    def cmd_vel_callback(self, msg: Twist):
        # Store linear and angular velocities.
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def get_wheel_speeds(self):
        # Differential-drive inverse kinematics:
        # wr = (2v + wL) / 2r, wl = (2v - wL) / 2r.
        wr = (2 * self.linear_velocity + self.angular_velocity * self.wheel_base) / (2 * self.wheel_radius)
        wl = (2 * self.linear_velocity - self.angular_velocity * self.wheel_base) / (2 * self.wheel_radius)
        return wr, wl

    def timer_callback(self):
        # Advance the robot state and publish pose and wheel speeds.
        current_time = self.get_clock().now()
        # `dt` is the sample time used by the Euler integrator.
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Euler integration of the nonholonomic model.
        self.x += self.linear_velocity * math.cos(self.theta) * dt
        self.y += self.linear_velocity * math.sin(self.theta) * dt
        self.theta += self.angular_velocity * dt

        # Keep heading bounded to [-pi, pi] for easier debugging/plotting.
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        wr, wl = self.get_wheel_speeds()

        self.wr_pub.publish(Float32(data=wr))
        self.wl_pub.publish(Float32(data=wl))
        self.x_pub.publish(Float32(data=self.x))
        self.y_pub.publish(Float32(data=self.y))
        self.theta_pub.publish(Float32(data=self.theta))

        # Publish the planar pose as a quaternion for downstream ROS tools.
        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time.to_msg()
        pose_msg.header.frame_id = self.odom_frame
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.z = math.sin(self.theta / 2.0)
        pose_msg.pose.orientation.w = math.cos(self.theta / 2.0)

        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotSim()

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
