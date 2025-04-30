import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import os

class GPTDescriptionNode(Node):
    def __init__(self):
        super().__init__('gpt_description')
        self.subscription = self.create_subscription(
            String,
            'image_base64',
            self.listener_callback,
            10
        )
        self.description_pub = self.create_publisher(String, 'gpt_description', 10)

        openai.api_key = os.getenv("GPT_API_KEY") 

    def listener_callback(self, msg):
        image_base64 = msg.data
        try:
            self.get_logger().info('GPT에게 이미지 설명 요청 중...')
            response = openai.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "당신은 자율 수상 드론 WAM-V를 위한 이미지 분석 AI입니다. "
                            "카메라는 모델 전면 기준 약 0.85m 앞 중심선에 위치하며, 수면으로부터 높이는 1.28m입니다. "
                            "이미지는 1920x1080 해상도, 수평시야각은 약 80도입니다. "
                            "시뮬레이터 UI 요소(초록색 수평선, 파란색 수직선 등)는 무시하세요. "
                            "부표 및 장애물 등의 객체를 JSON 형식으로 요약하세요. "
                            "예: {\"objects\": [{\"color\": \"red\", \"position\": \"center\", \"distance\": \"medium\"}]}"
                        )
                    },
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", 
                             "text": "이 이미지에서 보이는 부표나 장애물 정보를 JSON으로 요약해줘."
                            },
                            {"type": "image_url", 
                             "image_url": {
                                 "url": f"data:image/png;base64,{image_base64}"
                                }
                            }
                        ]
                    }
                ],
                max_tokens=200,
                temperature=0.2,
            )
            description = response.choices[0].message.content.strip()
            self.description_pub.publish(String(data=description))
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
