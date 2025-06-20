import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import os
import json
import re  # 코드블록 제거를 위한 정규표현식 사용

class GPTNode(Node):
    def __init__(self):
        super().__init__('gpt_node')
        self.subscription = self.create_subscription(
            String,
            'image_base64',
            self.listener_callback,
            10
        )
        self.command_pub = self.create_publisher(String, 'gpt_command', 10)
        self.trigger_pub = self.create_publisher(String, 'save_image_trigger', 10)

        openai.api_key = os.getenv("GPT_API_KEY")

    def listener_callback(self, msg):
        image_base64 = msg.data
        try:
            # Step 1: GPT로 이미지 설명 받기
            response = openai.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "You are the image analysis AI for an autonomous surface drone (WAM-V). "
                            "The input is a single image captured by the front-facing camera at a resolution of 1920x1080. "
                            "The camera is positioned 0.85 meters forward and 1.28 meters above the water surface from the drone's center. "
                            "The horizontal field of view (FOV) of the camera is 85 degrees. "
                            "Ignore any UI elements visible in the image (e.g., green horizontal lines, blue vertical lines)."
                            "The two gray shapes at the bottom of the image are the drone's engines. These are not objects to be analyzed, but if any external object is close enough to touch them, it must be considered a collision."
                            "Summarize **all major objects** appearing in the image in JSON format. "
                            "Each object must include : "
                            "- 'type': Must be either obstacle or rubber_duck."
                            "- 'color': dominant visible color. "
                            "- 'angle': Relative to the drone’s center; 0 degrees is straight ahead, negative values indicate left, and positive values indicate right (unit: degrees). "
                            "- 'distance': Integer from 0 (very close, touching the drone) to 10 (very far, barely visible at horizon)."
                            "Distance estimation rules:"
                            "- Distance must be based primarily on the vertical position of the lowest visible point of the object in the image."
                            "Objects with their bottom edge located low (near the engines) in the image are considered close (e.g., distance 0-3)."
                            "Objects with their bottom edge located high (near the top of the image) are considered far (e.g., distance 7-10)."
                            "Use object size only as a secondary clue if vertical position is ambiguous."
                            "This value must reflect the **physical distance from the drone to the object**, not the spacing between multiple objects."
                            "Do not infer distance based on visual tricks or object types."
                            "The output must strictly follow this JSON structure and must not include any natural language description: { \"object\": [ ... ] }"
                        )
                    },
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "text",
                                "text": "Summarize all obstacles (including buoys) and rubber ducks visible in this image in JSON format."
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
                temperature=0.3
            )

            # 응답 수신
            description_json = response.choices[0].message.content.strip()
            self.get_logger().info(f'[1단계] GPT 설명 응답 수신됨:\n{description_json}')

            # 코드블록 제거 (```json ... ```)
            if description_json.startswith("```json") or description_json.startswith("```"):
                description_json = re.sub(r"^```json\s*|\s*```$", "", description_json.strip())

            # JSON 파싱
            parsed = json.loads(description_json)
            object_list = parsed["object"]

            # Step 2: GPT로 행동 명령 판단
            decision_response = openai.chat.completions.create(
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
                            "4. Search for the rubber_duck if not visible: "
                            "- If no rubber_duck is present in the input:"
                            "   - Use the last_seen_duck_direction input value, which can be 'left', 'right', or 'none'."
                            "   - If 'left', output 'left'."
                            "   - If 'right', output 'right'."
                            "   - If 'none', default to 'left'."
                            "**You must output only one of the following words and nothing else**: "
                            "'forward', 'left', 'right', 'stop'"
                        )
                    },
                    {
                        "role": "user",
                        "content": f"Input objects JSON: {json.dumps(object_list)}"
                    }
                ],
                max_tokens=7,
                temperature=0.1
            )

            command = decision_response.choices[0].message.content.strip()
            self.command_pub.publish(String(data=command))
            self.get_logger().info(f'[2단계] GPT 명령 생성: {command}')

            # Step 3: 다음 이미지 저장 트리거
            self.trigger_pub.publish(String(data='next'))
            self.get_logger().info('[trigger] 다음 이미지 저장 요청 전송됨')

        except Exception as e:
            self.get_logger().error(f'GPT 처리 중 오류 발생: {e}')
            self.trigger_pub.publish(String(data='next'))


def main(args=None):
    rclpy.init(args=args)
    node = GPTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
