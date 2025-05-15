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
            response = openai.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "You are the navigation decision AI for an autonomous surface drone (WAM-V). "
                            "The input is object information analyzed from the front-facing camera, provided in JSON format. "
                            "This JSON lists all major objects in front of the drone, and each object includes the following attributes: "
                            "type (either obstacle or rubber_duck), color, angle (in degrees), and distance. "
                            "The drone considers the rubber_duck (yellow rubber duck) as the target, and must approach it **without collision** and stop upon reaching it. "
                            "   - If the rubber_duck is not present in the image: move the drone either left or right to make the rubber_duck appear in the image. "
                            "     Always issue a direction different from the previous one during search mode. "
                            "   - If the rubber_duck is present in the image: prioritize avoiding obstacles while approaching the rubber_duck. "
                            "       - If the rubber_duck has a positive angle: output right. "
                            "       - If the rubber_duck has a negative angle: output left. "
                            "Objects of type 'obstacle' are hazards that must not be collided with. "
                            "The angle is measured with 0 degrees at the front of the drone, negative to the left, and positive to the right. "
                            "If the distance to an obstacle is 5 or less, consider evasive action; if the distance is 3 or less, you must issue left or right to avoid it. "
                            "If the distance to the rubber_duck is 3 or less, output stop and do not issue any further commands. "
                            "The drone must make navigation decisions to reach the target without touching obstacles. "
                            "The output command must be one of the following words, without punctuation: forward, left, right, stop."
                        )



                    },
                    {
                        "role": "user",
                        "content": f"Input: {description_json}\nTo reach the target above while avoiding obstacles, choose the most appropriate direction in a single word. The output must be one of: forward, left, right, stop."
                    }
                ],
                max_tokens=7
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