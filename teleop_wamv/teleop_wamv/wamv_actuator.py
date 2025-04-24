import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64

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
            thrust.data = 30.0
            self.left_pub.publish(thrust)
            self.right_pub.publish(thrust)
        elif command == 'left':
            self.left_pub.publish(Float64(data=15.0))
            self.right_pub.publish(Float64(data=30.0))
        elif command == 'right':
            self.left_pub.publish(Float64(data=30.0))
            self.right_pub.publish(Float64(data=15.0))
        elif command == 'stop':
            self.left_pub.publish(Float64(data=-10.0))
            self.right_pub.publish(Float64(data=-10.0))
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
