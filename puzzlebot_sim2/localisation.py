import math
import rclpy
import numpy as np
from nav_msgs.msg import Odometry
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray


class Localisation(Node):

    def __init__(self):
        super().__init__('localisation')
        
        # Params
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.declare_parameter('theta0', 0.0)

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.19)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        
        # Noise params 
        self.declare_parameter('kr',0.002)
        self.declare_parameter('kl',0.002)

        # Read Params
        self.x = self.get_parameter('x0').value
        self.y = self.get_parameter('y0').value
        self.theta = self.get_parameter('theta0').value
        # Read odom 
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # Dynamic noise params
        self.kr = self.get_parameter('kr').value
        self.kl = self.get_parameter('kl').value
        # self.dt = 1.0/self._rate
        
        # Estimated robot state.
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.yaw = self.theta

        # Measured wheel angular speeds.
        self.wr = 0.0
        self.wl = 0.0
        
        # Covariance matrix P for pose [x, y, theta]
        self.P = np.zeros((3, 3))

        # EKF/ArUco parameters
        self.declare_parameter('use_ekf', False)
        self.declare_parameter('ekf_r_dist', 0.05)
        self.declare_parameter('ekf_r_bearing', 0.05)
        self.declare_parameter('marker_ids', [0])
        self.declare_parameter('marker_pos_x', [1.0])
        self.declare_parameter('marker_pos_y', [0.0])

        self.use_ekf = bool(self.get_parameter('use_ekf').value)
        self.ekf_r_dist = float(self.get_parameter('ekf_r_dist').value)
        self.ekf_r_bearing = float(self.get_parameter('ekf_r_bearing').value)
        marker_ids = self.get_parameter('marker_ids').value
        marker_pos_x = self.get_parameter('marker_pos_x').value
        marker_pos_y = self.get_parameter('marker_pos_y').value
        self.known_markers = {int(i): (float(x), float(y)) for i, x, y in zip(marker_ids, marker_pos_x, marker_pos_y)}
        self.latest_detection = None
        self.new_detection = False
        self.aruco_sub = self.create_subscription(
            Float32MultiArray, '/aruco/detections', self.aruco_callback, 10)

        # Static Q values
        # self.A = 0.00005
        # self.B = 0.000005
        # self.C = 0.0001

        self.last_time = self.get_clock().now()
        self.dt = 0.0
        self.odom_msg = Odometry()

        # Wheel speeds behave like sensor signals, so use sensor-data QoS.
        self.create_subscription(Float32, 'wr', self.wr_callback, qos.qos_profile_sensor_data)
        self.create_subscription(Float32, 'wl', self.wl_callback, qos.qos_profile_sensor_data)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("Localisation node initialized.")

    def aruco_callback(self, msg):
        if len(msg.data) < 3:
            return
        self.latest_detection = (int(msg.data[0]), float(msg.data[1]), float(msg.data[2]))
        self.new_detection = True

    def ekf_correct(self, marker_id, z_dist, z_bearing):
        if marker_id not in self.known_markers:
            return
        mx, my = self.known_markers[marker_id]
        dx = mx - self.x
        dy = my - self.y
        h_dist = math.sqrt(dx**2 + dy**2)
        if h_dist < 1e-6:
            return
        h_bearing = math.atan2(dy, dx) - self.yaw
        h_bearing = math.atan2(math.sin(h_bearing), math.cos(h_bearing))
        y_dist = z_dist - h_dist
        y_bearing = z_bearing - h_bearing
        y_bearing = math.atan2(math.sin(y_bearing), math.cos(y_bearing))
        H = np.array([
            [-dx / h_dist,     -dy / h_dist,      0.0],
            [ dy / h_dist**2,  -dx / h_dist**2,  -1.0]
        ])
        R = np.diag([self.ekf_r_dist**2, self.ekf_r_bearing**2])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        delta = K @ np.array([y_dist, y_bearing])
        self.x   += delta[0]
        self.y   += delta[1]
        self.yaw += delta[2]
        self.yaw  = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.P    = (np.eye(3) - K @ H) @ self.P
        self.P    = 0.5 * (self.P + self.P.T)
        
    def wr_callback(self, msg:Float32):
        # Store the latest right wheel speed.
        self.wr = msg.data

    def wl_callback(self, msg:Float32):
        # Store the latest left wheel speed.
        self.wl = msg.data

    def get_robot_vel(self):
        # Linear and angular velocity from wheel speeds.
        self.linear_velocity = (self.wheel_radius * (self.wr + self.wl)) / 2.0
        self.angular_velocity = (self.wheel_radius * (self.wr - self.wl)) / self.wheel_base

    def update_pose(self):
        # Integrate the dead-reckoning model using the current velocity.
        current_time = self.get_clock().now()
        self.dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        self.x += self.linear_velocity * math.cos(self.yaw) * self.dt
        self.y += self.linear_velocity * math.sin(self.yaw) * self.dt
        self.yaw += self.angular_velocity * self.dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        return current_time
    
    # New function: covariance propagation 
    """def update_covariance(self, v, w, dt): 
        # Jacobian matrix
        J_h = np.array([
            [1, 0, -v * dt * np.sin(self.yaw)],
            [0, 1, v * dt * np.cos(self.yaw)],
            [0, 0, 1]
        ])
        # Static Q noise matrix 
        Q = np.array([
            [self.A, self.B, self.B],
            [self.B, self.A, self.B],
            [self.B, self.B, self.C]
        ])
        # covariance propagation 
        self.P = J_h @ self.P @ J_h.T + Q
        """
    def update_covariance(self, v, w, dt):
        # Wheel linear displacements during this time step
        dr = self.wheel_radius * self.wr * dt
        dl = self.wheel_radius * self.wl * dt

        # Differential-drive increments
        dc = (dr + dl) / 2.0
        dtheta = (dr - dl) / self.wheel_base

        # Use the previous/local heading approximation for the covariance model
        theta_mid = self.yaw + dtheta / 2.0

        # Jacobian of the motion model with respect to the previous pose
        J_h = np.array([
            [1.0, 0.0, -dc * math.sin(theta_mid)],
            [0.0, 1.0,  dc * math.cos(theta_mid)],
            [0.0, 0.0,  1.0]
        ])

        # Jacobian of the motion model with respect to wheel displacements [dr, dl]
        J_delta = np.array([
            [
                0.5 * math.cos(theta_mid) - (dc / (2.0 * self.wheel_base)) * math.sin(theta_mid),
                0.5 * math.cos(theta_mid) + (dc / (2.0 * self.wheel_base)) * math.sin(theta_mid)
            ],
            [
                0.5 * math.sin(theta_mid) + (dc / (2.0 * self.wheel_base)) * math.cos(theta_mid),
                0.5 * math.sin(theta_mid) - (dc / (2.0 * self.wheel_base)) * math.cos(theta_mid)
            ],
            [
                1.0 / self.wheel_base,
                -1.0 / self.wheel_base
            ]
        ])

        # Dynamic wheel-noise matrix.
        # More wheel displacement -> more uncertainty.
        Sigma_delta = np.array([
            [self.kr * abs(dr), 0.0],
            [0.0, self.kl * abs(dl)]
        ])

        # Dynamic process noise in pose space
        Q = J_delta @ Sigma_delta @ J_delta.T

        # Covariance propagation
        self.P = J_h @ self.P @ J_h.T + Q

        # Numerical cleanup: force symmetry
        self.P = 0.5 * (self.P + self.P.T)

    def fill_odom_message(self, current_time):
        # Create a new Odometry msg
        odom_msg = Odometry()
        # Fill with robot pose
        # Get current time
        odom_msg.header.stamp = current_time.to_msg()
        # Use standard TF frame names without a leading slash.
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # x, y, z positions (m)
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Convert planar yaw into a quaternion without external deps.
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        # Twist 
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.angular.z = self.angular_velocity

        self.odom_msg = odom_msg
        
    def publish_odometry(self):
        # Add covariance matrix to odom msg 
        odom_msg = self.odom_msg
        odom_msg.pose.covariance = [0.0] * 36
        
        odom_msg.pose.covariance[0] = self.P[0, 0]  # Variance in x
        odom_msg.pose.covariance[7] = self.P[1, 1]  # Variance in y
        odom_msg.pose.covariance[35] = self.P[2, 2] # Variance in theta    
        odom_msg.pose.covariance[1] = self.P[0, 1]  # Covariance between x and y
        odom_msg.pose.covariance[6] = self.P[1, 0]  # Covariance between y and x
        odom_msg.pose.covariance[5] = self.P[0, 2]  # Covariance between x and theta
        odom_msg.pose.covariance[30] = self.P[2, 0]  # Covariance between theta and x
        odom_msg.pose.covariance[11] = self.P[1, 2]  # Covariance between y and theta
        odom_msg.pose.covariance[31] = self.P[2, 1]  # Covariance between theta and y
        
        self.odom_pub.publish(odom_msg)

    def timer_callback(self):
        # Update the pose estimate and publish the odometry message.
        self.get_robot_vel()
        current_time = self.update_pose()
        if self.dt <= 0.0:
            return
        self.update_covariance(self.linear_velocity, self.angular_velocity, self.dt)
        # EKF correction step
        if self.use_ekf and self.new_detection:
            mid, zd, zb = self.latest_detection
            self.ekf_correct(mid, zd, zb)
            self.new_detection = False
        self.fill_odom_message(current_time)
        self.publish_odometry()


def main(args=None):
    rclpy.init(args=args)
    node = Localisation()

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
