import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def move_one_circle(self, linear_vel=2.0, angular_vel=1.0):
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel

        duration = 2 * math.pi / angular_vel
        start_time = self.get_clock().now()

        self.get_logger().info('Starting one circle...')
        while (self.get_clock().now() - start_time).nanoseconds * 1e-9 < duration:
            self.publisher_.publish(msg)

        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Done!')

    def draw_s_shape(self, linear_vel=1.0, segment_duration=1.0):
        """
        Draw S shape with all segments equal length
        Pattern: izquierda, abajo, derecha, abajo, izquierda
        """
        msg = Twist()
        
        self.get_logger().info('Drawing S shape with equal segments...')
        
        # 1. Izquierda
        msg.linear.x = -linear_vel
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self._publish_for_duration(msg, segment_duration)

        # 2. Abajo
        msg.linear.x = 0.0
        msg.linear.y = -linear_vel
        msg.angular.z = 0.0
        self._publish_for_duration(msg, segment_duration)

        # 3. Derecha
        msg.linear.x = linear_vel
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self._publish_for_duration(msg, segment_duration)

        # 4. Abajo
        msg.linear.x = 0.0
        msg.linear.y = -linear_vel
        msg.angular.z = 0.0
        self._publish_for_duration(msg, segment_duration)

        # 5. Izquierda
        msg.linear.x = -linear_vel
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self._publish_for_duration(msg, segment_duration)

        # Stop
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('S shape completed!')

    def _publish_for_duration(self, msg, duration):
        """Helper method to publish a message for a specific duration"""
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds * 1e-9 < duration:
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()

    try:
        while True:
            cmd = input("Press 'c' to move one circle, 's' to draw S shape, or 'q' to quit: ").strip().lower()
            if cmd == 'c':
                node.move_one_circle()
            elif cmd == 's':
                node.draw_s_shape()
            elif cmd == 'q':
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()