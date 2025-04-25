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
                            "당신은 자율 수상 드론을 제어하는 AI입니다. "
                            "사용자로부터 객체의 위치와 거리 정보를 담은 JSON을 입력받아 드론이 이동할 방향을 판단합니다. "
                            "결과는 반드시 다음 네 개의 단어 중 하나로 응답해야 합니다. ' '또는 " "는 응답에 나타나면 안 됩니다. left ,  right , stop ,  forward  "
                        )
                    },
                    {
                        "role": "user",
                        "content": f"설명: {description_json}\n이 정보를 바탕으로 드론이 안전하게 고무오리까지 도달하려면 어떤 방향으로 이동해야 하나요?"
                    }
                ],
                max_tokens=100
            )
            command_json = response.choices[0].message.content.strip()
            self.publisher.publish(String(data=command_json))
            self.get_logger().info(f'GPT 명령(JSON): {command_json}')
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
