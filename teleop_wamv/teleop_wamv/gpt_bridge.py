import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import os
import json

class GPTBridge(Node):
    def __init__(self):
        super().__init__('gpt_bridge')
        self.subscription = self.create_subscription(
            String, 
            'gpt_description', 
            self.listener_callback, 
            10
        )
        self.publisher = self.create_publisher(String, 'gpt_command', 10)
        self.trigger_pub = self.create_publisher(String, 'save_image_trigger', 10)

        openai.api_key = os.getenv("GPT_API_KEY")

    def listener_callback(self, msg):
        try:
            description_json = msg.data
            self.get_logger().info(f'GPT에게 JSON 설명 전달 중: {description_json}')
            response = openai.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "당신은 자율 수상 드론을 제어하는 명령 결정기입니다. "
                            "입력은 JSON 포맷이며, 고무오리(target)와 장애물(obstacle)의 거리(m) 및 방향(deg)이 포함됩니다. "
                            "드론은 고무오리까지 도달해야 하며, 장애물과는 1m 이상 떨어져야 합니다. "
                            "출력은 반드시 다음 네 가지 중 하나의 단어로만 하세요: forward, left, right, stop. "
                        )
                    },
                    {
                        "role": "user",
                        "content": f"설명: {description_json}\n이 정보를 바탕으로 드론이 다른 물체와 충돌하기 않고 고무오리까지 도달하려면 어떤 방향으로 이동해야 하나요?"
                    }
                ],
                max_tokens=100
            )
            command_json = response.choices[0].message.content.strip()
            self.publisher.publish(String(data=command_json))
            self.get_logger().info(f'GPT 명령(JSON): {command_json}')

            # 다음 이미지 저장 트리거 발행
            self.trigger_pub.publish(String(data='next'))
            self.get_logger().info('[trigger] 다음 이미지 저장 요청 전송됨')
            
        except Exception as e:
            self.get_logger().error(f'GPT 요청 중 오류 발생: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GPTBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()