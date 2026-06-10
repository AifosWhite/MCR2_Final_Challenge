#!/usr/bin/env python3
"""
Calibración de encoders del Puzzlebot.
Uso:
  ros2 run final_challenge wheel_calibration --ros-args -p mode:=linear
  ros2 run final_challenge wheel_calibration --ros-args -p mode:=angular
"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class WheelCalibration(Node):
    def __init__(self):
        super().__init__('wheel_calibration')

        self.declare_parameter('mode', 'linear')   # 'linear' o 'angular'
        self.declare_parameter('linear_speed',  0.12)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('distance_m',    1.0)   # metros a recorrer
        self.declare_parameter('angle_deg',     360.0) # grados a girar

        mode          = str(self.get_parameter('mode').value)
        linear_speed  = float(self.get_parameter('linear_speed').value)
        angular_speed = float(self.get_parameter('angular_speed').value)
        distance_m    = float(self.get_parameter('distance_m').value)
        angle_deg     = float(self.get_parameter('angle_deg').value)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(0.5)

        if mode == 'linear':
            duration = distance_m / linear_speed
            self.get_logger().info(
                f'Moviendo {distance_m} m a {linear_speed} m/s ({duration:.2f} s)...')
            self._move(linear_speed, 0.0, duration)
            self.get_logger().info('Listo. Mide la distancia real con una cinta.')

        elif mode == 'angular':
            import math
            angle_rad = math.radians(angle_deg)
            duration  = angle_rad / angular_speed
            self.get_logger().info(
                f'Girando {angle_deg}° a {angular_speed} rad/s ({duration:.2f} s)...')
            self._move(0.0, angular_speed, duration)
            self.get_logger().info('Listo. Mide cuántos grados giró realmente.')

        else:
            self.get_logger().error(
                f'Modo desconocido: "{mode}". Usa linear o angular.')

    def _move(self, vx, wz, duration):
        msg = Twist()
        msg.linear.x  = float(vx)
        msg.angular.z = float(wz)
        t0 = time.time()
        while time.time() - t0 < duration:
            self.pub.publish(msg)
            time.sleep(0.05)
        self.pub.publish(Twist())  # stop


def main(args=None):
    rclpy.init(args=args)
    node = WheelCalibration()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()