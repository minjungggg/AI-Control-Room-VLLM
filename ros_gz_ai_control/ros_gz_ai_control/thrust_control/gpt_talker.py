import os
import openai
import rclpy
from rclpy.node import Node

class GPTImageDescriber(Node):
    def __init__(self):
        super().__init__('gpt_image_describer')

        # OpenAI API 키 설정 (환경 변수에서)
        openai.api_key = os.getenv("GPT_API_KEY")

        # 이미지 경로
        self.image_path = os.path.expanduser('~/saved_images/latest_image.png')

        # 주기적으로 이미지 분석 시도 (5초 간격)
        self.timer = self.create_timer(5.0, self.analyze_image)
        self.get_logger().info("GPT Image Describer Node Started")

    def analyze_image(self):
        if not os.path.exists(self.image_path):
            self.get_logger().warn(f"Image not found at: {self.image_path}")
            return

        try:
            with open(self.image_path, "rb") as img_file:
                image_data = self._to_base64(img_file.read())

                response = openai.chat.completions.create(
                    model="gpt-4o",
                    messages=[
                        {
                            "role": "system",
                            "content": (
                                "이 이미지는 바다 위에 떠 있는 쌍동선 모델의 전면 중앙에 부착된 카메라로 촬영되었습니다. "
                                "카메라는 모델 전면 기준으로 약 0.85m 앞쪽, 중심선상(y=0), 수면으로부터 약 1.28m 높이에 위치"
                                "카메라는 1920x1080 해상도를 가지며, 수평 시야각은 약 80도입니다. "
                                "주어진 정보를 바탕으로 이미지 내 부표들의 상대 위치, 특징, 그리고 대략적인 거리 추정을 간결하고 빠르게 응답해 주세요."
                            )
                        },
                        {
                            "role": "user",
                            "content": [
                                {"type": "text", "text": "이미지를 바탕으로 부표들의 상대 위치와 특징, 거리 추정을 해줘. 응답은 간결하고 5초 이내로 부탁해."},
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
                    temperature=0.2,
                    top_p=0.5,
                    stream=False
                )

            description = response.choices[0].message.content
            self.get_logger().info(f"GPT 분석 결과:\n{description}")

        except Exception as e:
            self.get_logger().error(f"GPT 요청 중 오류 발생: {str(e)}")

    def _to_base64(self, image_bytes):
        import base64
        return base64.b64encode(image_bytes).decode('utf-8')


def main(args=None):
    rclpy.init(args=args)
    node = GPTImageDescriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()