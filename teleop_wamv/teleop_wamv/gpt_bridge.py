import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import os

class GPTBridge(Node):
    def __init__(self):
        super().__init__('gpt_bridge')
        self.subscription = self.create_subscription(
            String,
            'image_base64',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(String, 'gpt_command', 10)
        self.client = openai.OpenAI(api_key=os.getenv("GPT_API_KEY"))

    def listener_callback(self, msg):
        image_base64 = msg.data
        try:
            self.get_logger().info('GPT에게 이미지 전달 중...')
            response = self.client.chat.completions.create(
                model="gpt-4-turbo",
                messages=[
                    {"role": "user", "content": [
                        {"type": "text", "text": "이 이미지를 분석해서 어떤 방향으로 이동해야 하는지 판단해서 명령어를 내려줘. 가능한 명령어는 forward, left, right, stop 중 하나야. 그리고 반드시 'command: ' 뒤에 명령어 하나만 줘."},
                        {"type": "image_url", "image_url": {
                            "url": f"data:image/png;base64,{image_base64}"
                        }}
                    ]}
                ],
                max_tokens=100
            )
            command = response.choices[0].message.content
            self.publisher.publish(String(data=command))
            self.get_logger().info(f'GPT 응답: {command}')
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
