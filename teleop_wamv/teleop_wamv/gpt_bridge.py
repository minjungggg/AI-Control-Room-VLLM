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
            'gpt_description',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(String, 'gpt_command', 10)
        self.client = openai.OpenAI(api_key=os.getenv("GPT_API_KEY"))

    def listener_callback(self, msg):
        try:
            self.get_logger().info('GPT에게 이미지 전달 중...')
            response = self.client.chat.completions.create(
                model="gpt-4-turbo",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "당신은 자율 수상 드론(WAM-V)을 제어하는 AI입니다."
                            "사용자로부터 이미지 설명을 입력받고, 이에 따라 드론이 진행해야 할 명령을 생성합니다."
                            "명령은 반드시 다음 중 하나여야 합니다: 'forward', 'left', 'right', 'stop'."
                        )
                    },
                    {
                        "role": "user", 
                        "content": f"설명: {msg.data}\n이 설명을 바탕으로 바다 위 물체에 부딪히지 않고 고무오리까지 도달할 수 있는 적절한 명령을 한 줄로 내려줘."
                        
                    }
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
