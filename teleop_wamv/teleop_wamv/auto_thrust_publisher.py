import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class AutoThrustPublisher(Node):
    def __init__(self):
        super().__init__('auto_thrust_publisher')
        self.left_pub = self.create_publisher(Float64, '/model/wamv_camera/joint/left_propeller_joint/cmd_thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/model/wamv_camera/joint/right_propeller_joint/cmd_thrust', 10)
        self.get_logger().info('Auto Thrust Publisher Started')
        self.timer = self.create_timer(0.1, self.publish_thrust)
        self.start_time = time.time()

    def publish_thrust(self):
        elapsed = time.time() - self.start_time
        left = Float64()
        right = Float64()

        if elapsed < 5:
            left.data = -100.0
            right.data = -100.0
        elif elapsed < 15:
            left.data = 100.0
            right.data = 100.0
        elif elapsed < 20:
            left.data = -100.0
            right.data = 100.0
        else:
            left.data = 0.0
            right.data = 0.0
            self.left_pub.publish(left)
            self.right_pub.publish(right)
            self.get_logger().info('Finished. Shutting down...')
            rclpy.shutdown()
            return

        self.left_pub.publish(left)
        self.right_pub.publish(right)

def main(args=None):
    rclpy.init(args=args)
    node = AutoThrustPublisher()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
