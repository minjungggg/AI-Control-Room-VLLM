import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty
import threading
from rcl_interfaces.msg import SetParametersResult

class KeyboardThrustController(Node):
    def __init__(self):
        super().__init__('keyboard_thrust_controller')
        # 퍼블리셔 생성
        self.left_pub = self.create_publisher(Float64, '/model/wamv_camera/joint/left_propeller_joint/cmd_thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/model/wamv_camera/joint/right_propeller_joint/cmd_thrust', 10)
        # 터미널 원래 설정 저장
        self.original_terminal_settings = termios.tcgetattr(sys.stdin)
        self.running = True
        self.reset_timer = None

        # 파라미터 선언 및 기본값 설정
        self.declare_parameter('forward_thrust', 20.0)
        self.declare_parameter('reverse_thrust', -20.0)
        self.declare_parameter('turn_thrust', 15.0)
        self.declare_parameter('reset_delay', 3.0)

        self.forward_thrust = self.get_parameter('forward_thrust').get_parameter_value().double_value
        self.reverse_thrust = self.get_parameter('reverse_thrust').get_parameter_value().double_value
        self.turn_thrust = self.get_parameter('turn_thrust').get_parameter_value().double_value
        self.reset_delay = self.get_parameter('reset_delay').get_parameter_value().double_value

        # 파라미터 변경 콜백 등록
        self.add_on_set_parameters_callback(self.parameter_callback)

        # 키보드 입력을 위한 별도 데몬 스레드 시작
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.keyboard_thread.start()

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'forward_thrust':
                self.forward_thrust = param.value
                self.get_logger().info(f'Updated forward_thrust to {self.forward_thrust}')
            elif param.name == 'reverse_thrust':
                self.reverse_thrust = param.value
                self.get_logger().info(f'Updated reverse_thrust to {self.reverse_thrust}')
            elif param.name == 'turn_thrust':
                self.turn_thrust = param.value
                self.get_logger().info(f'Updated turn_thrust to {self.turn_thrust}')
            elif param.name == 'reset_delay':
                self.reset_delay = param.value
                self.get_logger().info(f'Updated reset_delay to {self.reset_delay}')
        return SetParametersResult(successful=True)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_terminal_settings)
        return key

    def publish_thrust(self, left_value, right_value):
        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = left_value
        right_msg.data = right_value
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        self.get_logger().info(f'Published thrusts - Left: {left_value}, Right: {right_value}')

    def schedule_reset(self):
        # 이전에 예약된 타이머가 있다면 취소
        if self.reset_timer is not None:
            self.reset_timer.cancel()
        # ROS2 타이머를 사용하여 한번만 실행할 콜백을 등록
        self.reset_timer = self.create_timer(self.reset_delay, self.reset_thrust_once)

    def reset_thrust_once(self):
        # Thrust를 0으로 재설정하고 타이머 취소
        self.publish_thrust(0.0, 0.0)
        if self.reset_timer is not None:
            self.reset_timer.cancel()
            self.reset_timer = None

    def keyboard_loop(self):
        try:
            while self.running:
                key = self.get_key()
                if key == 'w':
                    self.publish_thrust(self.forward_thrust, self.forward_thrust)
                    self.schedule_reset()
                elif key == 's':
                    self.publish_thrust(self.reverse_thrust, self.reverse_thrust)
                    self.schedule_reset()
                elif key == 'a':
                    self.publish_thrust(-self.turn_thrust, self.turn_thrust)
                    self.schedule_reset()
                elif key == 'd':
                    self.publish_thrust(self.turn_thrust, -self.turn_thrust)
                    self.schedule_reset()
                elif key == 'q':
                    # 종료 명령 시 thrust 0 전송 후 종료
                    self.publish_thrust(0.0, 0.0)
                    self.running = False
                    import rclpy
                    rclpy.shutdown()
                    self.get_logger().info('Exiting...')
                    break
                else:
                    self.get_logger().info(f'Unknown key: {key}')
        except Exception as e:
            self.get_logger().error(f'Keyboard loop Exception: {e}')
        finally:
            # 어떤 이유로 루프가 끝나더라도 터미널 설정 복원
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_terminal_settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardThrustController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.keyboard_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
