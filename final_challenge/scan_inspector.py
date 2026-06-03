#!/usr/bin/env python3
"""
scan_inspector.py — Determina el lidar_yaw_offset correcto.

USO:
  1. Pon el robot con el FRENTE FÍSICO apuntando a una pared a ~25-35 cm
     (el frente = donde está la cámara / la parte delantera del chasis)
  2. Corre: python3 scan_inspector.py
  3. Lee el veredicto al final
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math

class ScanInspector(Node):
    def __init__(self):
        super().__init__('scan_inspector')
        self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.done = False
        self.get_logger().info("Esperando un mensaje de /scan...")

    def cb(self, msg):
        if self.done:
            return
        self.done = True

        ranges = np.array(msg.ranges, dtype=float)
        # Reemplaza inf/nan con valor grande
        ranges = np.where(np.isfinite(ranges) & (ranges > 0.05), ranges, 99.0)

        N = len(ranges)
        min_idx  = int(np.argmin(ranges))
        min_dist = ranges[min_idx]
        angle_raw = msg.angle_min + min_idx * msg.angle_increment

        # Normalizar a [-pi, pi]
        def norm(a):
            return math.atan2(math.sin(a), math.cos(a))

        angle_with_offset_pi  = norm(angle_raw - math.pi)
        angle_with_offset_0   = norm(angle_raw - 0.0)

        # Índices de las regiones "front" con cada offset
        # front = ángulo 0 en el frame del robot
        # Con offset=0:   frente del robot = índice donde angle_raw ≈ 0
        # Con offset=π:   frente del robot = índice donde angle_raw ≈ π

        front_idx_if_offset_0  = int(round((0.0         - msg.angle_min) / msg.angle_increment)) % N
        front_idx_if_offset_pi = int(round((math.pi     - msg.angle_min) / msg.angle_increment)) % N

        dist_at_front_0  = ranges[front_idx_if_offset_0]
        dist_at_front_pi = ranges[front_idx_if_offset_pi]

        print()
        print("=" * 55)
        print("  SCAN INSPECTOR — resultado")
        print("=" * 55)
        print(f"  Total puntos:      {N}")
        print(f"  angle_min raw:     {math.degrees(msg.angle_min):.1f}°")
        print(f"  angle_max raw:     {math.degrees(msg.angle_max):.1f}°")
        print(f"  angle_increment:   {math.degrees(msg.angle_increment):.4f}°")
        print()
        print(f"  OBJETO MÁS CERCANO")
        print(f"    Índice:          {min_idx}  (de 0 a {N-1})")
        print(f"    Distancia:       {min_dist:.3f} m")
        print(f"    Ángulo raw:      {math.degrees(angle_raw):.1f}°  ({angle_raw:.4f} rad)")
        print()
        print(f"  SI offset = 0.0:")
        print(f"    El controller ve ese objeto a: {math.degrees(angle_with_offset_0):.1f}°")
        print(f"    Distancia 'al frente' del robot: {dist_at_front_0:.3f} m")
        print()
        print(f"  SI offset = π (3.14159):")
        print(f"    El controller ve ese objeto a: {math.degrees(angle_with_offset_pi):.1f}°")
        print(f"    Distancia 'al frente' del robot: {dist_at_front_pi:.3f} m")
        print()
        print("=" * 55)
        print("  VEREDICTO")
        print("=" * 55)

        # La pared está al frente físico → la distancia real al frente debe ser la mínima
        # El offset correcto es el que hace que la distancia "al frente" coincida con min_dist

        tol = 0.15  # 15 cm de tolerancia

        frente_correcto_0  = abs(dist_at_front_0  - min_dist) < tol
        frente_correcto_pi = abs(dist_at_front_pi - min_dist) < tol

        if frente_correcto_pi and not frente_correcto_0:
            print()
            print("  ✅  offset = 3.14159 (π) es el CORRECTO")
            print(f"     Con offset=π, el robot 've' la pared al frente: {dist_at_front_pi:.3f} m")
            print(f"     Con offset=0, el frente del robot mide: {dist_at_front_0:.3f} m (lejos)")
            print()
            print("  → Mantén lidar_yaw_offset: 3.14159 en el YAML")

        elif frente_correcto_0 and not frente_correcto_pi:
            print()
            print("  ✅  offset = 0.0 es el CORRECTO")
            print(f"     Con offset=0, el robot 've' la pared al frente: {dist_at_front_0:.3f} m")
            print(f"     Con offset=π, el frente del robot mide: {dist_at_front_pi:.3f} m (lejos)")
            print()
            print("  → Cambia lidar_yaw_offset: 0.0 en el YAML")

        elif frente_correcto_0 and frente_correcto_pi:
            print()
            print("  ⚠️  Ambos valores dan resultado similar.")
            print(f"     Distancia mínima: {min_dist:.3f} m")
            print(f"     frente con 0:     {dist_at_front_0:.3f} m")
            print(f"     frente con π:     {dist_at_front_pi:.3f} m")
            print()
            print("  → Pon el robot MÁS CERCA de la pared (<25 cm) y repite.")

        else:
            print()
            print("  ⚠️  La pared más cercana NO está al frente del robot,")
            print(f"     o está muy lejos ({min_dist:.2f} m).")
            print()
            print("  → Pon el robot con el FRENTE apuntando a la pared a <35 cm")
            print("     y repite. La pared debe ser el objeto más cercano.")

        print()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ScanInspector()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
