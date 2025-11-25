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

        # time needed to complete one circle = 2π / angular_vel
        duration = 2 * math.pi / angular_vel
        start_time = self.get_clock().now()

        self.get_logger().info('Starting one circle...')
        while (self.get_clock().now() - start_time).nanoseconds * 1e-9 < duration:
            self.publisher_.publish(msg)

        # stop after finishing
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Done!')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()

    try:
        while True:
            cmd = input("Press 'c' to move one circle or 'q' to quit: ").strip().lower()
            if cmd == 'c':
                node.move_one_circle()
            elif cmd == 'q':
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()