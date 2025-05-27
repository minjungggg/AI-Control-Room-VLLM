import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String, Bool
from rcl_interfaces.msg import SetParametersResult

class RobotMotionController(Node):
    def __init__(self):
        super().__init__('robot_motion_controller')
        self.shutdown_requested = False
        
        # Thrust 제어용 퍼블리셔 생성
        self.left_pub = self.create_publisher(Float64, '/model/wamv_camera/joint/left_propeller_joint/cmd_thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/model/wamv_camera/joint/right_propeller_joint/cmd_thrust', 10)
        
        # busy 상태를 알리기 위한 퍼블리셔 생성 (다른 노드가 구독)
        self.thrust_busy_pub = self.create_publisher(Bool, 'thrust_busy', 10)
        self.thrust_is_busy = False  # 작업 상태 플래그

        # 'move_robot' 토픽 구독 (메시지는 String 타입)
        self.create_subscription(String, 'move_direction', self.move_callback, 10)

        # 'stop_robot' 토픽 구독 (메시지는 String 타입)
        self.create_subscription(String, 'stop_robot', self.stop_callback, 10)
        
        # 파라미터 선언 및 기본값 설정
        self.declare_parameter('forward_thrust', 10.0)
        self.declare_parameter('reverse_thrust', -10.0)
        self.declare_parameter('turn_thrust', 15.0)
        self.declare_parameter('reset_delay', 2.0)

        self.forward_thrust = self.get_parameter('forward_thrust').get_parameter_value().double_value
        self.reverse_thrust = self.get_parameter('reverse_thrust').get_parameter_value().double_value
        self.turn_thrust = self.get_parameter('turn_thrust').get_parameter_value().double_value
        self.reset_delay = self.get_parameter('reset_delay').get_parameter_value().double_value

        # 파라미터 변경 콜백 등록
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # 리셋 타이머 초기화
        self.reset_timer = None

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

    def publish_thrust_busy(self, status: bool):
        busy_msg = Bool()
        busy_msg.data = status
        self.thrust_busy_pub.publish(busy_msg)
        self.get_logger().info(f'Publishing thrust_busy = {status}')
    
    def publish_thrust(self, left_value, right_value):
        # 명령 실행 전 busy 상태 설정 및 발행
        self.thrust_is_busy = True
        self.publish_thrust_busy(True)
        
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
        # reset_delay 후 한번만 실행할 콜백 예약 (한 번 실행하고 타이머 취소)
        self.reset_timer = self.create_timer(self.reset_delay, self.reset_thrust_once)

    def reset_thrust_once(self):
        # Thrust를 0으로 reset
        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = 0.0
        right_msg.data = 0.0
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        self.get_logger().info('Published thrusts - Left: 0.0, Right: 0.0')

        # 타이머 취소
        if self.reset_timer is not None:
            self.reset_timer.cancel()
            self.reset_timer = None 
        # 작업 완료 후 busy 상태 자동 해제
        self.thrust_is_busy = False
        self.publish_thrust_busy(False)
        self.get_logger().info("Reset complete, node is no longer busy.")

    def stop_callback(self, msg: String):
        self.get_logger().info("Stop command received. Resetting thrust and shutting down.")
        self.reset_thrust_once()
        self.shutdown_requested = True
        self.get_logger().info("Shutting down the node.")

    def move_callback(self, msg: String):
        command = msg.data.strip().lower()  # 공백 제거 및 소문자 변환 처리
        if self.shutdown_requested:
            self.get_logger().warn("Shutdown 요청 상태에서 move 명령 무시.")
            return

        if self.thrust_is_busy:
            self.get_logger().info("이미 명령 수행 중입니다. 새 명령어 무시합니다.")
            return

        if command == 'w':
            self.publish_thrust(self.forward_thrust, self.forward_thrust)
            self.schedule_reset()
        elif command == 's':
            self.publish_thrust(self.reverse_thrust, self.reverse_thrust)
            self.schedule_reset()
        elif command == 'a':
            self.publish_thrust(-self.turn_thrust, self.turn_thrust)
            self.schedule_reset()
        elif command == 'd':
            self.publish_thrust(self.turn_thrust, -self.turn_thrust)
            self.schedule_reset()
        else:
            self.get_logger().warn(f'Unknown command received: {command}')
            return


def main(args=None):
    rclpy.init(args=args)
    node = RobotMotionController()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.shutdown_requested:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if node.reset_timer is not None:
            node.reset_timer.cancel()
            node.reset_timer = None
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()