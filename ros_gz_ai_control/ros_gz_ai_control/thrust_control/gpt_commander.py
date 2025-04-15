import os
import base64
import threading
import time
import openai
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class GPTImageRobotController(Node):
    def __init__(self):
        super().__init__('gpt_image_robot_controller')

        openai.api_key = os.getenv("GPT_API_KEY")
        self.image_path = os.path.expanduser('~/saved_images/latest_image.png')

        self.move_pub = self.create_publisher(String, 'move_robot', 10)
        self.direction_pub = self.create_publisher(String, 'move_direction', 10)
        self.stop_pub = self.create_publisher(String, 'stop_robot', 10)
        self.thrust_busy_sub = self.create_subscription(Bool, 'thrust_busy', self.thrust_busy_callback, 10)

        self.thrust_is_busy = False
        self.processing = False
        self.timer = self.create_timer(5.0, self.timer_callback)

        self.get_logger().info("GPT Image Robot Controller Node Started")

    def thrust_busy_callback(self, msg: Bool):
        self.thrust_is_busy = msg.data

    def timer_callback(self):
        if self.processing:
            self.get_logger().debug("현재 처리 중, 새로운 분석 요청 무시")
            return

        # 🔒 thrust_is_busy 방어
        if self.thrust_is_busy:
            self.get_logger().info("로봇이 busy 상태이므로 analyze_image 실행 안함.")
            return

        self.processing = True
        threading.Thread(target=self.analyze_image, daemon=True).start()

    def analyze_image(self):
        try:
            # 🔒 thrust_is_busy 방어
            if self.thrust_is_busy:
                self.get_logger().info("로봇이 busy 상태이므로 analyze_image 중단.")
                return

            if not os.path.exists(self.image_path):
                self.get_logger().warn(f"Image not found at: {self.image_path}")
                return

            with open(self.image_path, "rb") as img_file:
                image_bytes = img_file.read()
                image_data = self._to_base64(image_bytes)

            response = openai.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "너는 로봇 제어 시스템을 위한 판단 역할을 맡고 있다. "
                            "이미지를 분석한 후 반드시 'stop', 'move' 중 하나의 명령어만을 응답해야 한다. "
                            "다른 어떠한 부가 설명 없이 오직 이 단어만을 반환해라."
                        )
                    },
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": "이미지를 분석해서 로봇 제어에 필요한 결정( stop, move )을 내려줘. 먼저 검정과 빨간 부표 사이를 지나가서 뒤에있는 노란 부표앞으로 가고싶어. 이미지에 부표가 없다면 stop 해줘."},
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": "data:image/png;base64," + image_data
                                }
                            }
                        ]
                    }
                ],
                max_tokens=10,
                temperature=0.2,
                top_p=0.5,
                stream=False
            )

            decision = response.choices[0].message.content.strip().lower()
            self.get_logger().info(f"GPT 결정: {decision}")

            self.process_decision(decision)

        except Exception as e:
            self.get_logger().error(f"GPT 요청 중 오류 발생: {str(e)}")
        finally:
            self.processing = False

    def process_decision(self, decision):
        # 🔒 thrust_is_busy 방어
        if self.thrust_is_busy:
            self.get_logger().info("로봇이 busy 상태이므로 process_decision 중단.")
            return

        if decision == "stop":
            self.execute_stop()
        elif decision == "move":
            self.execute_move()
        else:
            self.get_logger().warn(f"예상치 않은 결정: '{decision}'. 명령 무시함.")

    def execute_stop(self):
        # 🔒 thrust_is_busy 방어
        if self.thrust_is_busy:
            self.get_logger().info("로봇이 busy 상태이므로 stop 명령 무시.")
            return

        self.get_logger().info("명령 'stop' 실행: 시스템을 안전하게 종료합니다.")
        self.stop_pub.publish(String(data="stop"))
        rclpy.shutdown()

    def execute_move(self):
        # 🔒 thrust_is_busy 방어
        if self.thrust_is_busy:
            self.get_logger().info("로봇이 busy 상태이므로 move 명령 무시.")
            return

        self.get_logger().info("명령 'move' 실행: 이동 명령 발행")
        self.move_pub.publish(String(data="move"))
        self.get_logger().info("이동 명령 'move' 전송 완료")

        try:
            direction = self.decide_direction_with_gpt()
            if direction in ['w', 'a', 's', 'd']:
                self.direction_pub.publish(String(data=direction))
                self.get_logger().info(f"방향 명령 '{direction}' 퍼블리시 완료")
            else:
                self.get_logger().warn(f"예상하지 못한 방향 응답: {direction}")
        except Exception as e:
            self.get_logger().error(f"방향 판단 GPT 호출 실패: {str(e)}")

    def decide_direction_with_gpt(self):
        with open(self.image_path, "rb") as img_file:
            image_data = self._to_base64(img_file.read())

        response = openai.chat.completions.create(
            model="gpt-4o",
            messages=[
                {
                    "role": "system",
                    "content": (
                        "너는 이동 로봇의 제어를 담당하고 있다. "
                        "이미지와 함께 명령이 주어지면, 로봇이 장애물에 부딪히지 않고 향해야 할 방향을 판단해야 한다. "
                        "반드시 'w', 'a', 's', 'd' 중 하나만 응답해야 한다. "
                        "'w'는 전진, 'a'는 좌회전, 's'는 후진, 'd'는 우회전을 의미한다. "
                        "다른 설명 없이 오직 이 문자 하나만 응답해라."
                    )
                },
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": "이동 방향을 w/a/s/d 중 하나로 판단해줘."},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": "data:image/png;base64," + image_data
                            }
                        }
                    ]
                }
            ],
            max_tokens=10,
            temperature=0.3,
            top_p=0.5,
            stream=False
        )

        direction = response.choices[0].message.content.strip().lower()
        return direction

    def _to_base64(self, image_bytes):
        return base64.b64encode(image_bytes).decode('utf-8')


def main(args=None):
    rclpy.init(args=args)
    node = GPTImageRobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt 수신, 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
