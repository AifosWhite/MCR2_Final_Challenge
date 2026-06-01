import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ReactiveNavigation(Node):
    def __init__(self):
        super().__init__('reactive_navigation_node')

        self.declare_parameter('waypoints_x', [1.15, 1.15, -1.15, -1.15])
        self.declare_parameter('waypoints_y', [-1.15, 1.15, 1.15, -1.15])
        self.declare_parameter('goal_tolerance', 0.12)
        self.declare_parameter('max_linear_speed', 0.14)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('wall_distance', 0.35)
        self.declare_parameter('front_clearance', 0.45)
        self.declare_parameter('side_clearance', 0.22)
        self.declare_parameter('emergency_stop_distance', 0.20)
        self.declare_parameter('wall_acquire_distance', 0.75)
        self.declare_parameter('wall_leave_clearance', 0.60)
        self.declare_parameter('bug_algorithm', 2)

        wx = list(self.get_parameter('waypoints_x').value)
        wy = list(self.get_parameter('waypoints_y').value)
        self.waypoints = [(float(x), float(y)) for x, y in zip(wx, wy)]
        if not self.waypoints:
            self.waypoints = [(1.15, -1.15)]
        self.goal_index = 0
        self.goal_x, self.goal_y = self.waypoints[self.goal_index]
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.max_v = float(self.get_parameter('max_linear_speed').value)
        self.max_w = float(self.get_parameter('max_angular_speed').value)
        self.wall_distance = float(self.get_parameter('wall_distance').value)
        self.front_clearance = float(self.get_parameter('front_clearance').value)
        self.side_clearance = float(self.get_parameter('side_clearance').value)
        self.emergency_stop_distance = float(self.get_parameter('emergency_stop_distance').value)
        self.wall_acquire_distance = float(self.get_parameter('wall_acquire_distance').value)
        self.wall_leave_clearance = float(self.get_parameter('wall_leave_clearance').value)
        self.bug_algorithm = int(self.get_parameter('bug_algorithm').value)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.scan = None
        self.state = 'go_to_goal'
        self.wall_side = 'right'
        self.wall_lock_count = 0
        self.hit_point = None
        self.best_goal_distance = math.inf

        # Deteccion de atasco / recovery (el loop corre a 10 Hz).
        self.stuck_ref_x = 0.0
        self.stuck_ref_y = 0.0
        self.stuck_cycles = 0
        self.recovery_cycles = 0
        self.recovery_dir = 1.0

        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'Navegacion Bug lista hacia ({self.goal_x:.2f}, {self.goal_y:.2f}).')

    def odom_callback(self, msg):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny, cosy)

    def scan_callback(self, msg):
        self.scan = msg

    def goal_callback(self, msg):
        self.goal_x = float(msg.pose.position.x)
        self.goal_y = float(msg.pose.position.y)
        self.waypoints = [(self.goal_x, self.goal_y)]
        self.goal_index = 0
        self.state = 'go_to_goal'
        self.hit_point = None
        self.best_goal_distance = math.inf
        self.get_logger().info(f'Nueva meta: ({self.goal_x:.2f}, {self.goal_y:.2f}).')

    def control_loop(self):
        if self.scan is None:
            self.publish_cmd(0.0, 0.0)
            return

        # --- Recovery de atasco -------------------------------------------
        # Giro en LAZO CERRADO: rota hacia el lado abierto y SALE apenas el
        # frente queda despejado (con histeresis), en vez de girar un numero
        # fijo de ciclos -> evita el overshoot que hace ver la pared de enfrente.
        if self.recovery_cycles > 0:
            self.recovery_cycles -= 1
            front = self.range_in_sector(0.0, math.radians(25.0))
            rear = self.range_in_sector(math.pi, math.radians(30.0))
            # Sale apenas hay ~0.40 m libres al frente (umbral ALCANZABLE en el
            # laberinto; 0.57 era imposible y lo dejaba girando sin escapar).
            if front > 0.40:
                self.recovery_cycles = 0
                self.reset_bug_state()
                self.stuck_ref_x, self.stuck_ref_y = self.x, self.y
                self.stuck_cycles = 0
                self.publish_cmd(0.04, 0.0)   # arranca derecho
                return
            # Retrocede (si hay hueco atras) mientras gira hacia el lado abierto:
            # crear espacio es lo que de verdad lo saca de una cuña/dead-end.
            back = -0.05 if rear > 0.25 else 0.0
            self.publish_cmd(back, 0.5 * self.recovery_dir)
            return

        # Mide progreso real (pose): si avanzamos algo, reinicia el contador.
        if math.hypot(self.x - self.stuck_ref_x, self.y - self.stuck_ref_y) > 0.05:
            self.stuck_ref_x, self.stuck_ref_y = self.x, self.y
            self.stuck_cycles = 0
        else:
            self.stuck_cycles += 1

        # ~7 s casi sin moverse -> recovery. Margen amplio para no interrumpir
        # un wall-follow lento que de hecho esta rodeando el obstaculo (Bug2).
        if self.stuck_cycles > 70:
            self.stuck_cycles = 0
            left = self.range_in_sector(math.pi / 2.0, math.radians(40.0))
            right = self.range_in_sector(-math.pi / 2.0, math.radians(40.0))
            self.recovery_dir = 1.0 if left > right else -1.0
            self.recovery_cycles = 45        # tope de seguridad (~4.5 s)
            self.reset_bug_state()           # salir de un follow_wall pegado
            self.get_logger().warn('Atascado -> recovery (giro hasta despejar).')
            self.publish_cmd(0.0, 0.45 * self.recovery_dir)
            return

        # Freno de emergencia (LiDAR, prioridad maxima): cubre un cono frontal
        # ancho Y los costados delanteros, porque al girar el robot roza la
        # pared con el costado aunque el frente este libre. Retrocede/gira hacia
        # el lado mas abierto, en cualquier estado.
        # Cono frontal ANGOSTO (±22°): asi las paredes laterales de un pasillo
        # estrecho no caen aqui y no disparan el freno todo el tiempo. La guardia
        # lateral es estrecha y solo salta ante contacto realmente inminente.
        front = self.range_in_sector(0.0, math.radians(22.0))
        side_l = self.range_in_sector(math.radians(50.0), math.radians(12.0))
        side_r = self.range_in_sector(math.radians(-50.0), math.radians(12.0))
        too_close_side = min(side_l, side_r)
        if front < self.emergency_stop_distance or too_close_side < 0.13:
            left = self.range_in_sector(math.pi / 2.0, math.radians(35.0))
            right = self.range_in_sector(-math.pi / 2.0, math.radians(35.0))
            turn = -1.0 if left < right else 1.0   # gira hacia donde hay mas hueco
            back = -0.05 if front < self.emergency_stop_distance else 0.0
            self.publish_cmd(back, 0.5 * turn)
            return

        dist, angle_error = self.goal_error()

        # Waypoint alcanzado?
        if dist < self.goal_tolerance:
            self.next_goal()
            return

        # Despacho segun el estado actual. ESTO es lo que faltaba antes: el
        # estado 'follow_wall' ahora es alcanzable y persistente, por lo que el
        # robot bordea la pared hasta que pueda progresar de verdad hacia la
        # meta, en vez de rebotar de inmediato hacia el mismo waypoint.
        if self.state == 'go_to_goal':
            self.go_to_goal(dist, angle_error)
        else:  # 'follow_wall'
            self.follow_wall(dist, angle_error)

    def goal_error(self):
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.hypot(dx, dy)
        desired = math.atan2(dy, dx)
        error = desired - self.theta
        error = math.atan2(math.sin(error), math.cos(error))
        return dist, error

    def next_goal(self):
        self.publish_cmd(0.0, 0.0)
        self.goal_index = (self.goal_index + 1) % len(self.waypoints)
        self.goal_x, self.goal_y = self.waypoints[self.goal_index]
        self.state = 'go_to_goal'
        self.wall_side = 'right'
        self.wall_lock_count = 0
        self.hit_point = None
        self.best_goal_distance = math.inf
        self.get_logger().info(
            f'Meta alcanzada. Siguiente WP{self.goal_index}: '
            f'({self.goal_x:.2f}, {self.goal_y:.2f}).'
        )

    def reset_bug_state(self):
        # Vuelve a go_to_goal y limpia el estado del bordeo de pared.
        self.state = 'go_to_goal'
        self.wall_lock_count = 0
        self.hit_point = None
        self.best_goal_distance = math.inf

    def go_to_goal(self, dist, angle_error):
        # Si hay algo al frente camino a la meta, cambia a seguimiento de pared.
        # La decision es puramente reactiva (LiDAR), no usa la pose estimada.
        front = self.range_in_sector(0.0, math.radians(25.0))
        if front < self.front_clearance:
            self.enter_follow_wall(dist)
            # ejecuta tambien una accion de wall-follow en este mismo ciclo
            d, e = self.goal_error()
            self.follow_wall(d, e)
            return

        # Banda muerta angular: ignora micro-errores de rumbo para no titilar
        # (la fuente principal del "baile").
        if abs(angle_error) < 0.05:
            angle_error = 0.0
        w = float(np.clip(1.3 * angle_error, -self.max_w, self.max_w))
        # Velocidad lineal con caida CONTINUA segun el desalineo (cos), en vez
        # de saltar entre 0 y avanzar: elimina el arranque-frenon.
        v = float(np.clip(0.8 * dist, 0.0, self.max_v))
        if abs(angle_error) > 0.6:        # solo frena en seco si esta muy torcido
            v = 0.0
        else:
            v *= max(0.0, math.cos(angle_error))
        self.publish_cmd(v, w)

    def enter_follow_wall(self, dist):
        self.state = 'follow_wall'
        self.hit_point = (self.x, self.y)
        self.best_goal_distance = dist
        self.wall_side = self.choose_wall_side()
        self.wall_lock_count = 8           # compromiso breve para no titilar
        self.get_logger().info(
            f'Obstaculo al frente -> follow_wall por la {self.wall_side}.')

    def follow_wall(self, dist, angle_error):
        if self.wall_lock_count > 0:
            self.wall_lock_count -= 1

        self.best_goal_distance = min(self.best_goal_distance, dist)

        # Sale de la pared en cuanto pueda progresar limpio hacia la meta.
        if self.can_leave_wall(dist, angle_error):
            self.state = 'go_to_goal'
            self.wall_lock_count = 0
            self.get_logger().info('Camino a la meta despejado -> go_to_goal.')
            self.go_to_goal(dist, angle_error)
            return

        front = self.range_in_sector(0.0, math.radians(20.0))
        if self.wall_side == 'left':
            side = self.range_in_sector(math.pi / 2.0, math.radians(20.0))
            diagonal = self.range_in_sector(math.pi / 4.0, math.radians(20.0))
            turn_sign = -1.0
        else:
            side = self.range_in_sector(-math.pi / 2.0, math.radians(20.0))
            diagonal = self.range_in_sector(-math.pi / 4.0, math.radians(20.0))
            turn_sign = 1.0

        if front < self.front_clearance:
            # Esquina interior: gira alejandote de la pared, sin avanzar.
            v = 0.0
            w = 0.32 * turn_sign
        elif diagonal < self.side_clearance:
            # Rozando la pared con la esquina delantera: sal despacio.
            v = 0.02
            w = 0.38 * turn_sign
        elif (not math.isfinite(side)) or side > self.wall_acquire_distance:
            # Perdimos la pared (esquina exterior): curva de regreso hacia ella.
            v = 0.04
            w = -0.4 * turn_sign
        else:
            # Si hay pared en AMBOS lados (pasillo angosto) -> centra, para no
            # pegarse al lado opuesto. Si no, sigue la pared a wall_distance.
            opp = self.range_in_sector(turn_sign * math.pi / 2.0, math.radians(20.0))
            if math.isfinite(opp) and (side + opp) < (2.0 * self.wall_distance + 0.20):
                center_err = opp - side          # >0: mas hueco del lado opuesto
                if abs(center_err) < 0.03:
                    center_err = 0.0
                w = float(np.clip(turn_sign * 1.0 * center_err, -0.4, 0.4))
            else:
                error = self.wall_distance - side
                if abs(error) < 0.03:
                    error = 0.0
                w = float(np.clip(-turn_sign * 1.0 * error, -0.45, 0.45))
            # Frena de forma continua segun lo cerca que este el frente.
            v = self.max_v * 0.6 * self._front_scale(front)

        self.publish_cmd(v, w)

    def _front_scale(self, front):
        # 1.0 con frente despejado, baja linealmente a 0 al acercarse al limite.
        lo = self.emergency_stop_distance
        hi = self.front_clearance
        if front >= hi:
            return 1.0
        if front <= lo:
            return 0.0
        return (front - lo) / (hi - lo)

    def choose_wall_side(self):
        left = self.range_in_sector(math.pi / 2.0, math.radians(30.0))
        right = self.range_in_sector(-math.pi / 2.0, math.radians(30.0))
        # Mantiene la pared en el lado mas cercano.
        return 'left' if left < right else 'right'

    def can_leave_wall(self, dist, angle_error):
        if self.wall_lock_count > 0:
            return False
        # Hay que estar mas o menos mirando a la meta...
        if abs(angle_error) > math.radians(35.0):
            return False
        # ...con el camino directo al frente despejado...
        if self.range_in_sector(0.0, math.radians(25.0)) < self.wall_leave_clearance:
            return False
        if self.range_in_sector(angle_error, math.radians(20.0)) < self.wall_leave_clearance:
            return False
        # Bug2: solo sale si estamos mas cerca de la meta que en el hit point.
        if self.bug_algorithm == 2:
            return dist < self.best_goal_distance - 0.05
        return True

    def range_in_sector(self, center, half_width):
        if self.scan is None:
            return math.inf

        best = math.inf
        angle = self.scan.angle_min
        for value in self.scan.ranges:
            if math.isfinite(value):
                delta = math.atan2(math.sin(angle - center), math.cos(angle - center))
                if abs(delta) <= half_width:
                    best = min(best, value)
            angle += self.scan.angle_increment
        return best

    def publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
