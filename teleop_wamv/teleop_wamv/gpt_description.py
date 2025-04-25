import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import os
import json

class GPTDescriptionNode(Node):
    def __init__(self):
        super().__init__('gpt_description')
        self.subscription = self.create_subscription(
            String,
            'image_base64',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(String, 'gpt_description', 10)
        openai.api_key = os.getenv("GPT_API_KEY") 

    def listener_callback(self, msg):
        image_base64 = msg.data
        try:
            self.get_logger().info('GPT에게 이미지 설명 요청 중...')
            response = openai.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {   "role": "system",
                        "content": (
                            "당신은 자율 수상 드론 WAM-V를 위한 이미지 분석 AI입니다. "
                            "카메라는 모델 전면 기준으로 약 0.85m 앞쪽 중심선상에 위치하고, 높이는 수면으로부터 약 1.28m입니다."
                            "이미지는 1920x1080 해상도이며, 수평시야각은 약 80도입니다."
                            "초록색 수평선, 파란색 수직선 등 시뮬레이터 UI 요소는 실제 물체가 아니므로 묘사에서 제외하세요."
                            "사용자는 카메라로 촬영한 이미지를 전송하며, 당신은 이미지에 등장하는 부표나 장애물 등의 객체를 감지하고, "
                            "그들의 위치 및 특징을 분석하여 아래와 같은 JSON 형식으로 응답해야 합니다. "
                            "예시: {\"objects\": [{\"color\": \"red\", \"position\": \"center\", \"distance\": \"medium\"}, ...]}"
                        )
                    },
                    {

                        "role": "user",
                        "content": [
                            {
                                "type": "text",
                                "text": ("이 이미지에서 보이는 부표나 장애물 정보를 JSON으로 요약해줘."
                                )
                            },
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/png;base64,{image_base64}"
                                }
                            }
                        ]
                    }
                ],
                max_tokens=300,
                temperature=0.5,
            )
            description = response.choices[0].message.content.strip()
            self.publisher.publish(String(data=description))
            self.get_logger().info(f'GPT JSON 설명: {description}')
        except Exception as e:
            self.get_logger().error(f'GPT 설명 요청 중 오류 발생: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GPTDescriptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
