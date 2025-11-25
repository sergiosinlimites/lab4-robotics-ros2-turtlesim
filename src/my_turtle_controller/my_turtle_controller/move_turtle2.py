import sys
import select
import termios
import tty
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.update_pose, 10)
        self.reset_client = self.create_client(Empty, 'reset')
        self.pose = Pose()
        self.get_logger().info('Controller Started. Waiting for pose...')
        # Wait for the first pose
        while self.pose.x == 0.0 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Pose received. Ready. Use Arrows to move. 1: SABP, 2: SFRM.')

    def update_pose(self, data):
        self.pose = data

    def reset_simulation(self):
        """Llama al servicio /reset de turtlesim"""
        if not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Servicio /reset no disponible')
            return
        
        req = Empty.Request()
        future = self.reset_client.call_async(req)
        # Esperar un poco a que se reinicie
        # No bloqueamos indefinidamente, solo lanzamos la petición
        self.get_logger().info('Reiniciando simulacion...')

    def stop(self):
        msg = Twist()
        self.publisher_.publish(msg)

    def send_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher_.publish(msg)

    def get_distance(self, goal_x, goal_y):
        return math.sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def rotate_to_angle(self, target_angle_rad, tolerance=0.01):
        """Gira la tortuga hasta orientarse al ángulo absoluto deseado"""
        msg = Twist()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            angle_diff = self.normalize_angle(target_angle_rad - self.pose.theta)
            
            if abs(angle_diff) < tolerance:
                self.stop()
                break
                
            msg.angular.z = 2.0 if angle_diff > 0 else -2.0
            # Slow down when close
            if abs(angle_diff) < 0.5:
                msg.angular.z = 0.5 if angle_diff > 0 else -0.5
            
            self.publisher_.publish(msg)
            time.sleep(0.01)

    def go_to_point(self, goal_x, goal_y, tolerance=0.1):
        """Navega línea recta hacia una coordenada X,Y"""
        msg = Twist()
        while self.get_distance(goal_x, goal_y) > tolerance and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            
            desired_angle = math.atan2(goal_y - self.pose.y, goal_x - self.pose.x)
            angle_diff = self.normalize_angle(desired_angle - self.pose.theta)

            # Simple P-Controller logic
            # Turn first if angle is too large
            if abs(angle_diff) > 0.2:
                msg.linear.x = 0.0
                msg.angular.z = 1.5 if angle_diff > 0 else -1.5
            else:
                msg.linear.x = 1.5 * self.get_distance(goal_x, goal_y)
                # Cap speed
                if msg.linear.x > 2.0: msg.linear.x = 2.0
                msg.angular.z = 2.0 * angle_diff
            
            self.publisher_.publish(msg)
            time.sleep(0.01)
        self.stop()

    def move_timed(self, linear, angular, duration):
        """Movimiento simple por tiempo (para dibujar trazos pequeños)"""
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        
        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            self.publisher_.publish(msg)
            # Importante: spin_once para mantener actualizada la pose
            rclpy.spin_once(self, timeout_sec=0.0) 
            time.sleep(0.02)
        self.stop()

    # --- Drawing Helpers ---
    def move_to_start_L(self, target_x, target_y):
        """Mueve a la posición objetivo en forma de L para evitar cruzar el dibujo"""
        self.get_logger().info(f'Yendo a inicio de secuencia: {target_x}, {target_y}')
        
        # Estrategia: Primero moverse en X lejos del centro si estamos cerca,
        # pero lo más seguro para ir a la esquina izquierda es:
        # 1. Ir a X objetivo (movimiento horizontal)
        # 2. Ir a Y objetivo (movimiento vertical)
        # Esto dibuja una L que generalmente bordea el área de trabajo central
        
        # Primero ajustamos X
        self.go_to_point(target_x, self.pose.y)
        # Luego ajustamos Y
        self.go_to_point(target_x, target_y)
        
        # Orientarse a 0
        self.rotate_to_angle(0.0)
        time.sleep(0.5)

    def reset_pen_pos(self, x, y):
        """Va al punto de inicio usando movimiento seguro en L"""
        self.move_to_start_L(x, y)

    # --- Letras Mejoradas ---
    # Cada letra asume que empieza en la esquina inferior izquierda de su caja (o donde se defina)
    # y termina donde sea, porque 'reset_pen_pos' corregirá la posición para la siguiente.

    def draw_S(self):
        # Start at bottom-left
        # Move up
        self.move_timed(1.0, 1.5, 2.0) # Curve up-left
        self.move_timed(1.0, -1.5, 2.0) # Curve up-right
        # This is approximate "S" shape

    def draw_A(self):
        # Start bottom-left
        self.rotate_to_angle(math.radians(75))
        self.move_timed(2.0, 0.0, 1.0) # Up slash
        self.rotate_to_angle(math.radians(-75))
        self.move_timed(2.0, 0.0, 1.0) # Down slash
        # Crossbar (hard to do perfectly without lifting pen, we back up)
        self.move_timed(-2.0, 0.0, 0.5) # Back up
        self.rotate_to_angle(math.radians(180))
        self.move_timed(1.0, 0.0, 0.5) # Cross

    def draw_B(self):
        # Start bottom-left
        self.rotate_to_angle(math.radians(90))
        self.move_timed(2.0, 0.0, 1.5) # Vertical up
        self.rotate_to_angle(math.radians(-90))
        # Top bump
        self.move_timed(1.5, -3.0, 1.0)
        self.rotate_to_angle(math.radians(180)) # Face left
        # Bottom bump
        self.rotate_to_angle(math.radians(0)) # Face right
        self.move_timed(1.5, -3.0, 1.2)

    def draw_P(self):
        # Start bottom-left
        self.rotate_to_angle(math.radians(90))
        self.move_timed(2.0, 0.0, 1.5) # Vertical up
        self.rotate_to_angle(math.radians(-90)) # Face right
        # Loop
        self.move_timed(1.5, -3.0, 1.1)

    def draw_F(self):
        self.rotate_to_angle(math.radians(90))
        self.move_timed(2.0, 0.0, 1.5) # Vertical up
        self.rotate_to_angle(math.radians(0))
        self.move_timed(1.5, 0.0, 0.5) # Top bar
        self.move_timed(-1.5, 0.0, 0.5) # Back
        self.move_timed(-0.0, 0.0, 0.2) # Nothing
        self.rotate_to_angle(math.radians(-90))
        self.move_timed(1.0, 0.0, 0.5) # Down a bit
        self.rotate_to_angle(math.radians(0))
        self.move_timed(1.0, 0.0, 0.5) # Middle bar

    def draw_R(self):
        # Draw P part
        self.draw_P()
        # Leg
        self.rotate_to_angle(math.radians(-45))
        self.move_timed(2.0, 0.0, 0.8)

    def draw_M(self):
        self.rotate_to_angle(math.radians(90))
        self.move_timed(2.0, 0.0, 1.5) # Up
        self.rotate_to_angle(math.radians(-45))
        self.move_timed(1.5, 0.0, 0.8) # Down diag
        self.rotate_to_angle(math.radians(45))
        self.move_timed(1.5, 0.0, 0.8) # Up diag
        self.rotate_to_angle(math.radians(-90))
        self.move_timed(2.0, 0.0, 1.5) # Down

    # --- Letras en secuencia continua ESTANDARIZADA ---
    # Altura: 4.0 | Ancho: ~2.5 | Espacio: 1.0
    # Velocidad lineal base: 2.0 m/s
    # Velocidad angular base: 2.0 rad/s (aprox pi/1.57 seg para 180 grados)

    def draw_S_continuous(self):
        # Inicia S (Arriba-Izquierda)
        self.get_logger().info('Dibujando S...')
        self.rotate_to_angle(math.radians(0))
        self.move_timed(2.0, 0.0, 0.5) # Pequeño avance derecha
        # Medio circulo hacia arriba (CCW)
        # Radio ~1.0 -> pi * r = 3.14 metros de arco.
        # Vel angular 2.0, Vel lineal 2.0 -> Radio = v/w = 1.0 perfecto.
        # Tiempo para pi radianes = pi / w = 3.14 / 2.0 = 1.57 seg
        # Dos medios circulos para cerrar la S
        self.move_timed(2.0, 2.0, 1.57) 
        self.move_timed(2.0, -2.0, 1.57)
        
        # Cola final S
        self.rotate_to_angle(math.radians(0))
        self.move_timed(2.0, 0.0, 0.75) # Avanzar un poco

    def draw_A_continuous(self):
        self.get_logger().info('Dibujando A...')
        # Viene de abajo de la S.
        
        # Inicia A
        # Hacia abajo
        self.rotate_to_angle(math.radians(-90))
        self.move_timed(2.0, 0.0, 2.0) 
        
        # Subir al medio para la barra
        self.rotate_to_angle(math.radians(90))
        self.move_timed(2.0, 0.0, 1.0) 

        # Derecha
        self.rotate_to_angle(math.radians(0))
        self.move_timed(2.0, 0.0, 1.0)


        # Abajo
        self.rotate_to_angle(math.radians(-90))
        self.move_timed(2.0, 0.0, 1.0)
        # Arriba para cerrar la A
        self.rotate_to_angle(math.radians(90))
        self.move_timed(2.0, 0.0, 2.0)

        # Izquierda devolver un poco para cerrar la A
        self.rotate_to_angle(math.radians(180))
        self.move_timed(2.0, 0.0, 1.0)
        # Volver a la derecha
        self.rotate_to_angle(math.radians(0))
        self.move_timed(2.0, 0.0, 1.0)

        # Termina Arriba-Derecha de la A

    def draw_B_continuous(self):
        self.get_logger().info('Dibujando B...')
        # Abajo
        self.rotate_to_angle(math.radians(-90))
        self.move_timed(2.0, 0.0, 2.0) # 4m abajo
        
        # Primera curva inferior (CCW)
        self.rotate_to_angle(math.radians(0)) # Mirar derecha
        self.move_timed(2.0, 0.0, 0.5) # 1m derecha
        # Semicirculo radio 1.0 (altura 2.0)
        self.move_timed(2.0, 2.0, 1.57)

        # El palo de en medio de la B
        self.rotate_to_angle(math.radians(180)) 
        self.move_timed(2.0, 0.0, 0.45) # 1m izquierda
        self.rotate_to_angle(math.radians(0)) # Mirar derecha
        self.move_timed(2.0, 0.0, 0.5)

        # Siguiente curva superior (CCW)
        self.rotate_to_angle(math.radians(0))
        self.move_timed(2.0, 2.0, 1.57)

        # El palo de arriba para cerrar la B
        self.rotate_to_angle(math.radians(180)) 
        self.move_timed(2.0, 0.0, 0.5) # 1m izquierda
        self.rotate_to_angle(math.radians(0)) # Mirar derecha
        self.move_timed(2.0, 0.0, 0.5)


    def draw_P_continuous(self):
        self.get_logger().info('Dibujando P...')
        # Abajo
        self.rotate_to_angle(math.radians(-90))
        self.move_timed(2.0, 0.0, 2.0) # 4m abajo

        # Subir para la curva
        self.rotate_to_angle(math.radians(90))
        self.move_timed(2.0, 0.0, 1.0)

        # Derecha y avanzar un poco
        self.rotate_to_angle(math.radians(0)) # Mirar derecha
        self.move_timed(2.0, 0.0, 0.5)
        
        # Curva P (CW) - Solo superior
        self.rotate_to_angle(math.radians(0))
        self.move_timed(2.0, 2.0, 1.57)

        # Cerrar la P
        self.rotate_to_angle(math.radians(180)) # Mirar derecha
        self.move_timed(2.0, 0.0, 0.5)
        
    def draw_SABP(self):
        # Ir al punto de inicio absoluto (Esquina superior izquierda)
        # S empieza arriba, así que Y alto.
        # Las otras letras empiezan más abajo, pero S es especial.
        # Ajuste: Empezar S alrededor de Y=6.0 (zona superior del mapa)
        self.move_to_start_L(1.0, 6.0) 
        
        self.draw_S_continuous()
        
        # Transición S -> A
        self.rotate_to_angle(math.radians(0))
        self.move_timed(2.0, 0.0, 0.25) # Pequeño avance derecha

        self.draw_A_continuous()

        # Transición A -> B
        self.rotate_to_angle(math.radians(0))
        self.move_timed(2.0, 0.0, 0.25) # Pequeño avance derecha

        self.draw_B_continuous()

        # Transición B -> P
        self.rotate_to_angle(math.radians(0))
        self.move_timed(2.0, 0.0, 0.75) # Pequeño avance derecha

        self.draw_P_continuous()
        # P termina abajo
        
        # Regresar a casa en L para no tachar
        self.move_to_start_L(5.5, 5.5)

    def draw_F_continuous(self):
        self.get_logger().info('Dibujando F...')
        # Asumimos que llega a la superior izquierda
        # Bajar vertical
        self.rotate_to_angle(math.radians(-90))
        self.move_timed(2.0, 0.0, 2.0) # 4m abajo

        # Subir hasta la mitad
        self.rotate_to_angle(math.radians(90))
        self.move_timed(2.0, 0.0, 1.0) # 2m arriba
        
        # Barra media
        self.rotate_to_angle(math.radians(0))
        self.move_timed(2.0, 0.0, 0.75) # 1.5m derecha
        self.rotate_to_angle(math.radians(180))
        self.move_timed(2.0, 0.0, 0.75) # Regresar

        # Subir hasta el final
        self.rotate_to_angle(math.radians(90))
        self.move_timed(2.0, 0.0, 1.0) # 2m arriba

        # Barra superior
        self.rotate_to_angle(math.radians(0))
        self.move_timed(2.0, 0.0, 1.0) # 2m derecha
        


    def draw_R_continuous(self):
        self.get_logger().info('Dibujando R...')
        # Ajuste vertical inicial (bajar hasta la base de la letra)
        self.rotate_to_angle(math.radians(-90))
        self.move_timed(2.0, 0.0, 2.0) # 4m abajo

        # Subir hasta la mitad
        self.rotate_to_angle(math.radians(90))
        self.move_timed(2.0, 0.0, 1.0) # 2m arriba

        # Pata diagonal
        # Primero un poco a la derecha
        self.rotate_to_angle(math.radians(0))
        self.move_timed(2.0, 0.0, 0.25) # 0.5m derecha
        # Angulo hacia abajo-derecha ~ -60 grados
        self.rotate_to_angle(math.radians(-60))
        self.move_timed(2.0, 0.0, 1.2) # Diagonal hacia abajo
        self.rotate_to_angle(math.radians(120))
        self.move_timed(2.0, 0.0, 1.2) # Diagonal hacia abajo
        
        # Curva P (CW) - Solo superior
        self.rotate_to_angle(math.radians(0))
        self.move_timed(2.0, 2.0, 1.57)
        self.rotate_to_angle(math.radians(180))
        self.move_timed(2.0, 0.0, 0.5);
        self.rotate_to_angle(math.radians(0))
        self.move_timed(2.0, 0.0, 0.5);


    def draw_M_continuous(self):
        self.get_logger().info('Dibujando M...')
        # Bajar vertical
        self.rotate_to_angle(math.radians(-90))
        self.move_timed(2.0, 0.0, 2.0) # 4m abajo
        # Subir de nuevo
        self.rotate_to_angle(math.radians(90))
        self.move_timed(2.0, 0.0, 2.0) # 4m arriba
        
        # Diagonal abajo-derecha hasta el centro
        self.rotate_to_angle(math.radians(-60))
        self.move_timed(2.0, 0.0, 1.2) # ~2.4m diagonal
        
        # Diagonal arriba-derecha
        self.rotate_to_angle(math.radians(60))
        self.move_timed(2.0, 0.0, 1.2) # ~2.4m diagonal
        
        # Bajar vertical
        self.rotate_to_angle(math.radians(-90))
        self.move_timed(2.0, 0.0, 2.0) # 4m abajo
        # Subir de nuevo
        self.rotate_to_angle(math.radians(90))
        self.move_timed(2.0, 0.0, 2.0) # 4m arriba

    def draw_SFRM(self):
        # Inicio de la secuencia en la parte inferior del mapa
        # Dibuja S (desde la parte baja, con altura aproximada de 4 unidades)
        self.move_to_start_L(1.0, 1.0) # Punto de inicio de la S inferior
        self.draw_S_continuous()
        
        # Transición S -> F
        # S termina abajo. F empieza abajo. Solo mover derecha.
        self.rotate_to_angle(math.radians(0))
        self.move_timed(2.0, 0.0, 0.25) # Espacio
        
        self.draw_F_continuous()
        # F termina abajo. R empieza abajo.
        
        # Transición F -> R
        self.rotate_to_angle(math.radians(0))
        self.move_timed(2.0, 0.0, 0.25) # Espacio
        
        self.draw_R_continuous()
        # R termina abajo. M empieza abajo.
        
        # Transición R -> M
        self.rotate_to_angle(math.radians(0))
        self.move_timed(2.0, 0.0, 1.0) # Espacio
        
        self.draw_M_continuous()
        
        # Casa: Subir primero para no tachar, luego al centro
        self.rotate_to_angle(math.radians(90)) # Mirar arriba
        self.move_timed(2.0, 0.0, 0.6) # Subir un poco
        self.move_to_start_L(5.5, 5.5)

    def draw_FULL(self):
        self.get_logger().info('Dibujando TODO (SABP + SFRM)...')
        self.draw_SABP()
        # draw_SABP termina en 5.5, 5.5
        # Ahora dibujamos SFRM
        self.draw_SFRM()

# --- Input Handling ---
settings = None

def save_terminal_settings():
    global settings
    if sys.stdin.isatty():
        settings = termios.tcgetattr(sys.stdin)

def restore_terminal_settings():
    if settings and sys.stdin.isatty():
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def get_key():
    if not sys.stdin.isatty():
        return None
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)
        return key
    return None

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    
    save_terminal_settings()
    print("Control: Arrows to move. 1: SABP, 2: SFRM, 3: FULL, c: Clear/Reset, h: Home, q: Quit")
    print("Individual letters: s, a, b, p, f, r, m")
    
    try:
        while rclpy.ok():
            key = get_key()
            if key:
                if key == 'q':
                    break
                elif key == 'c':
                    print("Limpiando y Reiniciando...")
                    node.reset_simulation()
                elif key == 'h':
                    print("Yendo a casa...")
                    node.move_to_start_L(5.54, 5.54)
                elif key == '\x1b[A': # Up
                    node.send_cmd(2.0, 0.0)
                elif key == '\x1b[B': # Down
                    node.send_cmd(-2.0, 0.0)
                elif key == '\x1b[C': # Right
                    node.send_cmd(0.0, -2.0)
                elif key == '\x1b[D': # Left
                    node.send_cmd(0.0, 2.0)
                elif key == '1':
                    print("Dibujando SABP...")
                    node.draw_SABP()
                    print("Fin SABP")
                elif key == '2':
                    print("Dibujando SFRM...")
                    node.draw_SFRM()
                    print("Fin SFRM")
                elif key == '3':
                    print("Dibujando TODO...")
                    node.draw_FULL()
                    print("Fin TODO")
                elif key.lower() == 's':
                    print("Dibujando S...")
                    node.draw_S_continuous()
                elif key.lower() == 'a':
                    print("Dibujando A...")
                    node.draw_A_continuous()
                elif key.lower() == 'b':
                    print("Dibujando B...")
                    node.draw_B_continuous()
                elif key.lower() == 'p':
                    print("Dibujando P...")
                    node.draw_P_continuous()
                elif key.lower() == 'f':
                    print("Dibujando F...")
                    node.draw_F_continuous()
                elif key.lower() == 'r':
                    print("Dibujando R...")
                    node.draw_R_continuous()
                elif key.lower() == 'm':
                    print("Dibujando M...")
                    node.draw_M_continuous()
                else:
                    node.stop()
            else:
                node.stop()
                
            rclpy.spin_once(node, timeout_sec=0.1)
            
    except Exception as e:
        print(e)
    finally:
        node.stop()
        restore_terminal_settings()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
