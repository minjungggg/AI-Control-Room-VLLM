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
                            "- type (either obstacle or rubber_duck)"
                            "- color"
                            "- angle (in degrees)"
                            "- distance (integer from 0 to 10) "
                            "The drone considers the rubber_duck (yellow rubber duck) as the target, and must approach it **without collision**, then stop upon reaching it. "
                            "Drone's navigation logic follows this strict priority: "
                            "1. Always prioritize 'rubber_duck' as the final target. Ignore 'obstacle' unless it is blocking the path to target"
                            "2. Avoid obstacle collisions:"
                            "- The drone must keep at least 3 distance from any obstacle."
                            "- If any obstacle is at distance ≤ 5 (any angle) : consider evasive maneuver (left or right) instead of going forward."
                            "- If any obstacle is at distance ≤ 3 and within angle ±40 degrees: output 'left' or 'right' to avoid collision immediately."
                            "3. Approach rubber_duck (the target):"
                            "- If no obstacle meets the avoidance criteria in step 1: "
                            "   - If rubber_duck angle > 0 : output 'right'"
                            "   - If rubber_duck angle < 0 : output 'left'"
                            "   - If rubber_duck angle = 0 : output 'forward'"
                            "   - If rubber_duck is present and distance ≤ 3 : output 'stop' immediately "

                            "3. Search for the rubber_duck if not visible: "
                            "- If no rubber_duck is present in the input:"
                            "   - Use the last_seen_duck_direction input value, which can be 'left', 'right', or 'none'."
                            "   - If 'left', output 'left'."
                            "   - If 'right', output 'right'."
                            "   - If 'none', default to 'left'."
                            "The angle is measured with 0 degrees directly ahead, negative to the left, and positive to the right. "
                            "**You must output only one of the following words and nothing else**: "
                            "'forward', 'left', 'right', 'stop'"
                            "Do not include punctuation, explanation, or prefixes."
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
            self.get_logger().info(f'GPT 명령: {command_json}')

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