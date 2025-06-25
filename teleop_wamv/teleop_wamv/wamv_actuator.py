import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
import time

class WamvActuator(Node):
    def __init__(self):
        super().__init__('wamv_actuator')
        self.subscription = self.create_subscription(
            String,
            'gpt_command',
            self.command_callback,
            10
        )
        self.left_pub = self.create_publisher(Float64, '/model/wamv_camera/joint/left_propeller_joint/cmd_thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/model/wamv_camera/joint/right_propeller_joint/cmd_thrust', 10)

    def command_callback(self, msg):
        command = msg.data.strip().lower()
        thrust = Float64()

        if command == 'forward':
            self.is_stopped = False
            thrust.data = 4.0
            self.left_pub.publish(thrust)
            self.right_pub.publish(thrust)
        elif command == 'left':
            self.is_stopped = False
            self.left_pub.publish(Float64(data=-5.0))
            self.right_pub.publish(Float64(data=7.0))
        elif command == 'right':
            self.is_stopped = False
            self.left_pub.publish(Float64(data=7.0))
            self.right_pub.publish(Float64(data=-5.0))
        elif command == 'stop':
            if not self.is_stopped:
                self.left_pub.publish(Float64(data=-70))
                self.right_pub.publish(Float64(data=-70))
                time.sleep(3.0)
                self.left_pub.publish(Float64(data=0.0))
                self.right_pub.publish(Float64(data=0.0))
                self.is_stopped = True
        else:
            self.get_logger().warn(f'알 수 없는 명령: {command}')
            return

        self.get_logger().info(f'명령 "{command}" 수행됨.')

def main(args=None):
    rclpy.init(args=args)
    node = WamvActuator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()