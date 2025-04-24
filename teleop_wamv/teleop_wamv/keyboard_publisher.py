import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.left_pub = self.create_publisher(Float64, '/model/wamv_camera/joint/left_propeller_joint/cmd_thrust',10)
        self.right_pub = self.create_publisher(Float64, '/model/wamv_camera/joint/right_propeller_joint/cmd_thrust',10)
        self.get_logger().info('Keyboard Publisher Started')        

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                left = Float64()
                right = Float64()

                if key == 'w':
                    left.data = 100.0
                    right.data = 100.0
                elif key == 's':
                    left.data = -100.0
                    right.data = -100.0
                elif key == 'a':
                    left.data = -100.0
                    right.data = 100.0
                elif key == 'd':
                    left.data = 100.0
                    right.data = -100.0
                elif key == 'q':
                    left.data = 0.0
                    right.data = 0.0
                    self.left_pub.publish(left)
                    self.right_pub.publish(right)
                    break
                else:
                    left.data = 0.0
                    right.data = 0.0

                self.left_pub.publish(left)
                self.right_pub.publish(right)
        except KeyboardInterrupt:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
