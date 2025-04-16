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
        self.description = None

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

            describe_response = openai.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "너는 항해중인 수중드론 제어 시스템을 위한 판단 역할을 맡고 있다."
                            "수중 드론은 쌍동선 형태이며, 이미지 하단에 보이는 두 개의 검은 돌출부는 드론 양쪽의 추진기로, 이 간격이 드론의 실제 가로 길이를 나타냅니다. 세로 길이는 이보다 약 2배 정도 길게 설계되어 있습니다. "
                            "현재 이미지를 촬영하고 있는 카메라의 시야각은 120도 이며, 드론 전면 부에서 약 0.85m 뒤에 위치에 있고, 이는 드론의 1/3 정도에 해당합니다."
                            "이미지를 바탕으로 주변 환경을 설명하라. 부표의 위치, 장애물, 항해 가능성 등에 대해 명확하고 자세하게 설명할 것."
                            "이미지 중앙 아래에 회색 물체는 드론의 추진기 앞부분이므로 장애물이 아님을 유의하라."
                        )
                    },
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": "이미지를 분석하고 주변 환경을 설명해줘."},
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": "data:image/png;base64," + image_data
                                }
                            }
                        ]
                    }
                ],
                max_tokens=300,
                temperature=0.5,
                top_p=0.5,
                stream=False
            )

            self.description = describe_response.choices[0].message.content.strip()
            self.get_logger().info(f"GPT 설명: {self.description}")

            # GPT의 결정에 따라 행동 결정
            decision_response = openai.chat.completions.create(
                model = "gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "너는 수중 드론의 항해를 제어하는 판단 시스템이다. 이전 설명을 기반으로 드론의 항해 방향을 판단해야 한다. "
                            "'stop' 또는 'move' 둘 중 하나만 응답하라. 설명 없이 단어 하나만 출력할 것."
                        )
                    },
                    {
                        "role": "assistant",
                        "content": self.description
                    },
                    {
                        "role": "user",
                        "content": "이 설명을 바탕으로 앞에 보이는 장애물을 지나 장애물 뒤로 가고싶어. 장애물 사이를 지나가도 좋고 장애물을 크게 피해가도 좋아."
                    }
                ],
                max_tokens=10,
                top_p=0.2,
                temperature=0.2,
                stream=False
            )
            decision = decision_response.choices[0].message.content.strip().lower()
            self.get_logger().info(f"GPT 결정: {decision}")
            if decision not in ["stop", "move"]:
                self.get_logger().warn(f"예상치 못한 결정: {decision}. 'stop' 또는 'move' 중 하나로 응답해야 함.")
                return
            # 결정에 따라 행동 수행
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
                        "너는 항해중인 이동 선박의 제어를 담당하고 있다. "
                        "수중 드론은 쌍동선 형태이며, 이미지 하단에 보이는 두 개의 검은 돌출부는 드론 양쪽의 추진기로, 이 간격이 드론의 실제 가로 길이를 나타냅니다. 세로 길이는 이보다 약 2배 정도 길게 설계되어 있습니다. 현재 이미지를 촬영하고 있는 카메라는 드론 전면에서 약 0.85m 앞으로 돌출된 위치에 있으며, 수면으로부터 약 1.2m 위에 설치되어 있습니다."
                        "이미지와 함께 명령이 주어지면, 로봇이 장애물에 부딪히지 않고 향해야 할 방향을 판단해야 한다."
                        "먼저 검정과 빨간 부표 사이를 통과할 것."
                        "반드시 'w', 'a', 's', 'd' 중 하나만 응답해야 한다. "
                        "'w'는 전진, 'a'는 좌회전, 's'는 후진, 'd'는 우회전을 의미한다. "
                        "다른 설명 없이 오직 이 문자 하나만 응답해라."
                    )
                },
                {
                    "role": "assistant",
                    "content": self.description
                },
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": "이동 방향을 w/a/s/d 중 하나로 판단해줘. 이미지에 부표가 없다면 a/d 로 회전해서 부표를 찾아야 해"},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": "data:image/png;base64," + image_data
                            }
                        }
                    ]
                }
            ],
            max_tokens=20,
            temperature=0.3,
            top_p=0.3,
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
