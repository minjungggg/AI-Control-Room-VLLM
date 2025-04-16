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
        threading.Thread(target=self.analyze_and_act, daemon=True).start()

    def analyze_and_act(self):
        try:
            if self.thrust_is_busy:
                self.get_logger().info("로봇이 busy 상태이므로 분석 중단.")
                return

            if not os.path.exists(self.image_path):
                self.get_logger().warn(f"이미지 파일이 존재하지 않음: {self.image_path}")
                return

            with open(self.image_path, "rb") as img_file:
                image_bytes = img_file.read()
                image_data = self._to_base64(image_bytes)

            # 1단계: GPT에게 이미지 설명 요청
            describe_response = openai.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "너는 항해중인 수중드론 제어 시스템을 위한 판단 역할을 맡고 있다. "
                            "수중 드론은 쌍동선 형태이며, 이미지 하단의 검은 돌출부는 드론 양쪽의 추진기로, 이 간격이 드론의 실제 가로 길이다. "
                            "세로 길이는 이보다 약 2배 정도이며, 카메라는 전면 기준 0.85m 뒤, 드론 길이의 약 1/3 지점에 위치한다. "
                            "이미지 중앙 하단에 있는 회색 물체는 드론의 앞부분으로 장애물이 아니다. "
                            "이미지를 분석해 부표의 위치, 장애물 유무, 항해 가능성 등을 명확하고 간단히 설명하라."
                        )
                    },
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": "이미지를 분석하고 주변 상황을 설명해줘."},
                            {"type": "image_url", "image_url": {"url": "data:image/png;base64," + image_data}}
                        ]
                    }
                ],
                max_tokens=300,
                temperature=0.5,
                top_p=0.5
            )

            description = describe_response.choices[0].message.content.strip()
            self.get_logger().info(f"[설명 결과] {description}")

            # 2단계: GPT에게 stop 또는 move 판단 요청
            decision_response = openai.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "너는 수중 드론의 항해 판단 시스템이다. 설명을 기반으로 'stop' 또는 'move' 중 하나만 응답하라. "
                            "설명 없이 반드시 단어 하나만 출력하라."
                        )
                    },
                    {"role": "assistant", "content": description},
                    {
                        "role": "user",
                        "content": (
                            "앞에 있는 장애물 뒤로 가고 싶어. 앞에 보이는 장애물을 지나 장애물 뒤로 가고싶어. 장애물 사이를 지나가도 좋고 장애물을 크게 피해가도 좋아."
                            "정지할 필요가 있다면 'stop', 통과 가능하면 'move' 중 하나만 말해."
                        )
                    }
                ],
                max_tokens=10,
                temperature=0.2,
                top_p=0.2
            )

            decision = decision_response.choices[0].message.content.strip().lower()
            self.get_logger().info(f"[판단 결과] {decision}")

            if decision not in ["stop", "move"]:
                self.get_logger().warn(f"예상치 못한 판단: {decision}")
                return

            if decision == "stop":
                self.get_logger().info("명령 'stop' 실행")
                self.stop_pub.publish(String(data="stop"))
                rclpy.shutdown()
                return

            # 3단계: move인 경우 방향 판단
            self.get_logger().info("명령 'move' 실행")
            self.move_pub.publish(String(data="move"))

            direction_response = openai.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "너는 수중 드론의 방향을 판단하는 시스템이다. 반드시 'w', 'a', 's', 'd' 중 하나로만 응답하라. "
                            "'w'=전진, 'a'=좌회전, 's'=후진, 'd'=우회전. 설명 없이 한 글자만 출력할 것."
                        )
                    },
                    {"role": "assistant", "content": description},
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": "장애물을 피하거나 통과할 수 있도록 w/a/s/d 중 하나로 판단해줘."}
                            # {"type": "image_url", "image_url": {"url": "data:image/png;base64," + image_data}}
                        ]
                    }
                ],
                max_tokens=10,
                temperature=0.3,
                top_p=0.3
            )

            direction = direction_response.choices[0].message.content.strip().lower()
            if direction in ['w', 'a', 's', 'd']:
                self.direction_pub.publish(String(data=direction))
                self.get_logger().info(f"[방향 결정] '{direction}' 퍼블리시 완료")
            else:
                self.get_logger().warn(f"예상치 못한 방향 응답: {direction}")

        except Exception as e:
            self.get_logger().error(f"[GPT 처리 중 오류] {str(e)}")

        finally:
            self.processing = False

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
