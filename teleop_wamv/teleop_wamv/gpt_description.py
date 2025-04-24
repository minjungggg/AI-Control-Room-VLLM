import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import os

class GPTDescriptionNode(Node):
    def __init__(self):
        super().__init__('gpt_description_node')
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
                model="gpt-4-turbo",
                messages=[
                    {   "role": "system",
                        "content": (
                            "이 이미지는 바다 위를 항해하는 자율 수상 드론(WAM-V)의 전면 카메라로 촬영된 것입니다."
                            "카메라는 모델 전면 기준으로 약 0.85m 앞쪽 중심선상에 위치하고, 높이는 수면으로부터 약 1.28m입니다."
                            "이미지는 1920x1080 해상도이며, 수평시야각은 약 80도입니다."
                            "초록색 수평선, 파란색 수직선 등 시뮬레이터 UI 요소는 실제 물체가 아니므로 묘사에서는 제외하지만, 물체들 간의 거리를 분석할 때 고려하세요."
                            " 바다 위에 떠 있는 물체들만 설명하고, 그들의 상대적 방향(왼쪽, 오른쪽, 중앙)과 거리를 표현하세요."
                            "객체들 간의 거리감을 설명하세요. 최대한 간결하고 명확하게 묘사하세요."
                        )
                    },
                    {

                        "role": "user",
                        "content": [
                            {
                                "type": "text",
                                "text": ("이미지를 바탕으로 부표들의 상대 위치와 특징, 거리 추정을 해줘. 객체가 여러개라면 왼쪽부터 오른쪽 방향으로 설명해줘. 응답은 간결하고 5초 이내로 부탁해."
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
                max_tokens=300
            )
            description = response.choices[0].message.content.strip()
            self.publisher.publish(String(data=description))
            self.get_logger().info(f'GPT 설명: {description}')
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
