import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import threading

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Variables para controlar el giro
        self.girando = False
        self.tiempo_transcurrido = 0.0
        self.velocidad_lineal = 2.0  # m/s
        self.velocidad_angular = 1.0  # rad/s
        self.tiempo_vuelta = (2 * math.pi) / self.velocidad_angular  # Tiempo para dar una vuelta completa
        
        # Timer que se ejecuta cada 0.1 segundos
        self.timer = self.create_timer(0.1, self.move_turtle)
        
        # Iniciar thread para leer teclado
        self.thread = threading.Thread(target=self.leer_teclado, daemon=True)
        self.thread.start()
        
        self.get_logger().info('Presiona C y Enter para dar un circulo')

    def leer_teclado(self):
        while rclpy.ok():
            try:
                tecla = input()
                if tecla.lower() == 'c' and not self.girando:
                    self.girando = True
                    self.tiempo_transcurrido = 0.0
                    self.get_logger().info('Iniciando circulo...')
            except:
                break

    def move_turtle(self):
        msg = Twist()
        
        if self.girando:
            # Hacer un círculo (avanzar y girar al mismo tiempo)
            msg.linear.x = self.velocidad_lineal
            msg.angular.z = self.velocidad_angular
            self.publisher_.publish(msg)
            
            # Incrementar tiempo
            self.tiempo_transcurrido += 0.1
            
            # Si completó la vuelta, detenerse
            if self.tiempo_transcurrido >= self.tiempo_vuelta:
                self.girando = False
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
                self.get_logger().info('Circulo completado. Presiona C para otro circulo')
        else:
            # Enviar comando de parada
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()