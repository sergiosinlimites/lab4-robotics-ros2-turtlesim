import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.current_pose = None

    def pose_callback(self, msg):
        self.current_pose = msg

    def wait_for_pose(self, timeout=5.0):
        """Wait until we receive the first pose message"""
        start_time = self.get_clock().now()
        while self.current_pose is None and (self.get_clock().now() - start_time).nanoseconds * 1e-9 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.current_pose is not None

    def normalize_angle(self, angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def turn_angle(self, angle_degrees):
        """
        Gira un ángulo específico en grados con alta precisión - método simple
        """
        if not self.wait_for_pose():
            self.get_logger().error('No pose received, cannot turn precisely')
            return False

        # Convertir grados a radianes
        angle_radians = math.radians(angle_degrees)
        
        # Calcular ángulo objetivo
        current_angle = self.normalize_angle(self.current_pose.theta)
        target_angle = self.normalize_angle(current_angle + angle_radians)
        
        self.get_logger().info(f'Turning {angle_degrees}° from {math.degrees(current_angle):.1f}° to {math.degrees(target_angle):.1f}°')
        
        msg = Twist()
        msg.linear.x = 0.0  # No movimiento lineal
        
        # Fase 1: Giro rápido hasta cerca del objetivo
        self.get_logger().info('Phase 1: Fast turn')
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds * 1e-9 < 1.5:  # Máximo 1.5 segundos
            if self.current_pose is None:
                continue
                
            current_angle = self.normalize_angle(self.current_pose.theta)
            error = self.normalize_angle(target_angle - current_angle)
            
            # Si estamos muy cerca, cambiar a fase lenta
            if abs(error) < 0.35:  # ~20°
                break
                
            # Giro rápido proporcional al error
            angular_cmd = max(min(error * 3.0, 2.0), -2.0)  # Máximo 2.0 rad/s
            msg.angular.z = angular_cmd
            self.publisher_.publish(msg)
            
            rclpy.spin_once(self, timeout_sec=0.01)
        
        # Fase 2: Giro lento y preciso para el ajuste final
        self.get_logger().info('Phase 2: Fine adjustment')
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds * 1e-9 < 3.0:  # Máximo 3 segundos
            if self.current_pose is None:
                continue
                
            current_angle = self.normalize_angle(self.current_pose.theta)
            error = self.normalize_angle(target_angle - current_angle)
            
            # Verificar si alcanzamos la precisión deseada
            if abs(error) < 0.0087:  # 0.5° de tolerancia
                break
                
            # Giro lento y constante para ajuste fino
            if abs(error) > 0.05:  # >2.8°
                msg.angular.z = 0.4 if error > 0 else -0.4
            else:  # Muy cerca
                msg.angular.z = 0.15 if error > 0 else -0.15
                
            self.publisher_.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        # Parada final - publicar múltiples veces para asegurar
        msg.angular.z = 0.0
        for _ in range(10):
            self.publisher_.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        # Verificación final
        if self.current_pose:
            current_angle = self.normalize_angle(self.current_pose.theta)
            final_error = math.degrees(self.normalize_angle(target_angle - current_angle))
            self.get_logger().info(f'Turn completed! Final angle: {math.degrees(current_angle):.1f}°')
            self.get_logger().info(f'Final error: {final_error:.2f}°')
            return abs(final_error) < 1.0  # Éxito si error < 1°
        
        return False

    def move_one_circle(self, linear_vel=2.0, angular_vel=1.0):
        """
        Mueve la tortuga en un círculo completo
        """
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel

        duration = 2 * math.pi / angular_vel
        start_time = self.get_clock().now()

        self.get_logger().info('Starting one circle...')
        while (self.get_clock().now() - start_time).nanoseconds * 1e-9 < duration:
            self.publisher_.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)

        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Circle completed!')

    def move_straight(self, distance=2.0, linear_vel=1.0):
        """
        Mueve la tortuga en línea recta una distancia específica
        """
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = 0.0

        duration = distance / linear_vel
        start_time = self.get_clock().now()

        self.get_logger().info(f'Moving straight {distance} meters...')
        while (self.get_clock().now() - start_time).nanoseconds * 1e-9 < duration:
            self.publisher_.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)

        msg.linear.x = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Straight movement completed!')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()

    try:
        while True:
            print("\n--- Turtle Control ---")
            print("Commands:")
            print("  'c' - Move in circle")
            print("  't' - Turn specific angle")
            print("  'm' - Move straight")
            print("  'q' - Quit")
            
            cmd = input("Enter command: ").strip().lower()
            
            if cmd == 'c':
                node.move_one_circle()
            elif cmd == 't':
                try:
                    angle = float(input("Enter angle in degrees (positive = counterclockwise, negative = clockwise): "))
                    node.turn_angle(angle)
                except ValueError:
                    print("Please enter a valid number")
            elif cmd == 'm':
                try:
                    distance = float(input("Enter distance in meters: "))
                    node.move_straight(distance)
                except ValueError:
                    print("Please enter a valid number")
            elif cmd == 'q':
                break
            else:
                print("Invalid command")
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()