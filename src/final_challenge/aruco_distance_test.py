#!/usr/bin/env python3
"""
aruco_distance_test.py — Prueba 1: Validar distancia y bearing del detector ArUco.

USO:
  1. Pon un ArUco pegado a la pared o apoyado en una caja, VERTICAL y bien iluminado.
  2. Mide con la regla la distancia desde el CENTRO del marcador hasta
     la LENTE de la cámara del robot. Anota ese valor.
  3. Corre el launch primero:
       ros2 launch puzzlebot_sim2 physical_challenge.launch.py nav:=false use_rviz:=false
  4. En otra terminal:
       python3 aruco_distance_test.py

El script muestra en tiempo real: ID, distancia reportada, bearing, y el error vs la distancia
que tú ingresas por teclado.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import threading
import time

# ─────────────────────────────────────────────
# TABLA DE MEDICIONES — llénala conforme pruebas
# ─────────────────────────────────────────────
# Formato: (distancia_real_m, descripción)
DISTANCIAS_A_PROBAR = [
    (0.20, "20 cm"),
    (0.30, "30 cm — largo de tu regla"),
    (0.40, "40 cm — dos largos de regla"),
    (0.60, "60 cm"),
]

class ArucoDistanceTest(Node):
    def __init__(self):
        super().__init__('aruco_distance_test')

        self.sub = self.create_subscription(
            Float32MultiArray,
            '/aruco/detections',
            self.cb,
            10
        )

        self.last_msg = None
        self.lock = threading.Lock()

        # Historial de lecturas para promediar
        self.history = []
        self.history_size = 10  # promedia las últimas 10 lecturas

        self.get_logger().info("Escuchando /aruco/detections ...")
        self.get_logger().info("Si no aparece nada, verifica que aruco_detector esté corriendo.")

    def cb(self, msg):
        if len(msg.data) < 3:
            return
        with self.lock:
            self.last_msg = {
                'id':      int(msg.data[0]),
                'dist':    float(msg.data[1]),
                'bearing': float(msg.data[2]),
            }
            self.history.append(self.last_msg['dist'])
            if len(self.history) > self.history_size:
                self.history.pop(0)

    def get_avg_dist(self):
        with self.lock:
            if not self.history:
                return None
            return sum(self.history) / len(self.history)

    def get_last(self):
        with self.lock:
            return self.last_msg.copy() if self.last_msg else None


def run_interactive(node):
    """Bucle interactivo: mide a diferentes distancias y reporta error."""

    resultados = []

    print()
    print("=" * 60)
    print("  PRUEBA 1 — Validación de distancia ArUco")
    print("=" * 60)
    print()
    print("  SETUP:")
    print("  • Pon el ArUco pegado a la pared, bien vertical")
    print("  • Robot mirando DE FRENTE al marcador (bearing ≈ 0)")
    print("  • Mide desde la LENTE de la cámara hasta el CENTRO del ArUco")
    print()
    print("  Presiona ENTER para tomar una lectura en cada distancia.")
    print("  Escribe 'q' para terminar y ver el resumen.")
    print()

    # Primero verificar que llegan detecciones
    print("  Verificando que llegan detecciones...")
    for i in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
        last = node.get_last()
        if last:
            print(f"  ✅ Detectando ArUco ID={last['id']} dist={last['dist']:.3f}m bear={math.degrees(last['bearing']):.1f}°")
            break
        if i == 29:
            print("  ❌ No llegan detecciones en /aruco/detections.")
            print("     Verifica: ros2 node list | grep aruco")
            print("     Verifica: ros2 topic hz /aruco/detections")
            return
    print()

    for dist_real, label in DISTANCIAS_A_PROBAR:
        print(f"  {'─'*50}")
        print(f"  📏 Pon el robot a {label} del marcador")
        print(f"     (usa la regla, mide desde la lente de la cámara)")
        cmd = input("  → Presiona ENTER cuando estés listo (o 'q' para terminar): ").strip().lower()
        if cmd == 'q':
            break

        # Limpiar historial y tomar 15 lecturas frescas
        node.history.clear()
        print(f"  Tomando lecturas", end="", flush=True)
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.15)
            if len(node.history) >= 10:
                break
            print(".", end="", flush=True)
        print()

        last = node.get_last()
        avg  = node.get_avg_dist()

        if last is None or avg is None:
            print("  ⚠️  No se recibieron detecciones. ¿El marcador está visible?")
            continue

        error_abs = avg - dist_real
        error_pct = (error_abs / dist_real) * 100.0

        bearing_deg = math.degrees(last['bearing'])

        print()
        print(f"  ID detectado:       {last['id']}")
        print(f"  Distancia REAL:     {dist_real*100:.0f} cm  ({dist_real:.3f} m)")
        print(f"  Distancia REPORTADA:{avg*100:.1f} cm  ({avg:.3f} m)  [prom. {len(node.history)} lecturas]")
        print(f"  Error:              {error_abs*100:+.1f} cm  ({error_pct:+.1f}%)")
        print(f"  Bearing:            {bearing_deg:.1f}°  (esperado ≈ 0° si estás de frente)")
        print()

        # Diagnóstico automático
        if abs(error_abs) < 0.05:
            print(f"  ✅ Error < 5 cm — distancia correcta")
        elif error_abs > 0.05:
            print(f"  ⚠️  Reporta MÁS lejos de lo real (+{error_abs*100:.1f} cm)")
            new_size = 0.096 * (dist_real / avg)
            print(f"     Prueba reducir marker_size_m a {new_size:.4f} m")
        else:
            print(f"  ⚠️  Reporta MÁS CERCA de lo real ({error_abs*100:.1f} cm)")
            new_size = 0.096 * (dist_real / avg)
            print(f"     Prueba aumentar marker_size_m a {new_size:.4f} m")

        if abs(bearing_deg) > 10:
            print(f"  ⚠️  Bearing = {bearing_deg:.1f}° (debería ser ≈0°)")
            print(f"     Ajusta la posición lateral del robot hasta que bearing ≈ 0")

        resultados.append({
            'label':      label,
            'real':       dist_real,
            'reportado':  avg,
            'error_cm':   error_abs * 100,
            'bearing':    bearing_deg,
        })
        print()

    # ─── RESUMEN FINAL ───
    if not resultados:
        print("  Sin resultados para resumir.")
        return

    print()
    print("=" * 60)
    print("  RESUMEN")
    print("=" * 60)
    print(f"  {'Distancia':>10}  {'Real (cm)':>10}  {'Report. (cm)':>13}  {'Error':>8}  {'Bearing':>8}")
    print(f"  {'─'*10}  {'─'*10}  {'─'*13}  {'─'*8}  {'─'*8}")
    for r in resultados:
        ok = "✅" if abs(r['error_cm']) < 5 else "⚠️ "
        print(f"  {r['label']:>10}  {r['real']*100:>10.1f}  {r['reportado']*100:>13.1f}  {r['error_cm']:>+7.1f}cm  {r['bearing']:>+7.1f}°  {ok}")

    errores = [abs(r['error_cm']) for r in resultados]
    avg_err = sum(errores) / len(errores)
    max_err = max(errores)

    print()
    print(f"  Error promedio: {avg_err:.1f} cm")
    print(f"  Error máximo:   {max_err:.1f} cm")
    print()

    # ─── RECOMENDACIÓN FINAL ───
    if avg_err < 3:
        print("  ✅ Calibración excelente. No necesitas cambiar marker_size_m.")
    elif avg_err < 6:
        print("  ✅ Calibración aceptable para el reto (error < 6 cm).")
    else:
        # Estimar marker_size_m óptimo
        ratios = [r['real'] / r['reportado'] for r in resultados]
        ratio_avg = sum(ratios) / len(ratios)
        new_marker = 0.096 * ratio_avg
        print(f"  ⚠️  Error promedio > 6 cm. Considera ajustar marker_size_m.")
        print(f"     marker_size_m actual:   0.096 m")
        print(f"     marker_size_m sugerido: {new_marker:.4f} m")
        print(f"     (en localisation_physical.yaml → aruco_detector → marker_size_m)")

    print()
    print("  Siguiente paso: Prueba 2 — validar el BEARING (signo y magnitud)")
    print("  Corre este script de nuevo y pon el robot LATERAL al marcador.")
    print()


def main():
    rclpy.init()
    node = ArucoDistanceTest()

    # Spin en background para no bloquear el input
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    try:
        run_interactive(node)
    except KeyboardInterrupt:
        print("\n  Interrumpido.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
