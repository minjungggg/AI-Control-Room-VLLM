import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import os
import json
import re

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
        self.description_pub = self.create_publisher(String, 'gpt_description', 10)
        self.trigger_pub = self.create_publisher(String, 'save_image_trigger', 10)

        openai.api_key = os.getenv("GPT_API_KEY")

    def listener_callback(self, msg):
        image_base64 = msg.data
        try:
            # Chain-of-Thought 프롬프트 (단계별 유도)
            prompt_system = (
                "You are the AI for an autonomous surface drone (WAM-V). "
                "You will perform two tasks based on a single input image captured by the drone's front camera "
                "(resolution: 1920x1080, camera at 0.85m forward and 1.28m height, 85° horizontal FOV). "
                "Ignore UI elements. The engines are gray shapes at the bottom and should not be considered as objects. "
                "If any object touches the engine, treat it as collision (distance=0). \n\n"

                "**Step 1: Object Detection**\n"
                "Identify all major objects in the image. For each object, output:\n"
                "- type: Must be either 'obstacle' or 'rubber_duck'\n"
                "- color: dominant visible color\n"
                "- angle: Relative to the drone’s center; 0 degrees is straight ahead, negative values indicate left, and positive values indicate right (unit: degrees)." 
                "The full image corresponds to a horizontal field of view from -42.5 degrees (far left edge) to +42.5 degrees (far right edge)." 
                "- distance: integer 0 (very close, touching the drone) to 10 (very far, barely visible at horizon)\n\n" 
                "Distance estimation rules: \n" 
                "- Distance must be based primarily on the vertical position of the lowest visible point of the object in the image.\n" 
                "- Objects with their bottom edge located low (near the engines) in the image are considered close (e.g., distance 0-3).\n" 
                "- Objects with their bottom edge located high (near the top of the image) are considered far (e.g., distance 7-10).\n" 
                "- Use object size only as a secondary clue if vertical position is ambiguous.\n\n" 
                "This value must reflect the **physical distance from the drone to the object**, not the spacing between multiple objects. \n" 
                "Do not infer distance based on visual tricks or object types.\n" 

                "**Step 2: Navigation Decision**\n"
                "The drone should approach the rubber_duck and stop when it is close (distance ≤ 3), while avoiding obstacles.\n" 
                "Navigation priority rules: \n"
                "1. Avoid obstacle collision: \n"
                "- Avoid any obstacle within distance ≤5.\n"
                "- If obstacle is within distance ≤ 3 and within angle ±20 degrees, perform immediate evasive action. \n"
                "- If obstacle is located closer to the left engine, turn right. \n"
                "- If obstacle is located closer to the right engine, turn left. \n" 
                "2. Prioritize 'rubber_duck'. Ignore obstacles unless they block the path.\n"
                "3. If the rubber_duck is visible:\n" 
                "- If the obstacle is closer than the rubber_duck (i.e., the distance value is smaller), then follow 'rule 1: Avoid obstacle collisions' first before targeting the duck. \n" 
                "- If the rubber_duck is closer than all obstacles, follow this approach logic:" 
                "   * duck's angle > 0: right\n" 
                "   * duck's angle < 0: left\n" 
                "   * duck's angle = 0: forward\n" 
                "   * duck's distance ≤ 3: stop\n" 
                "4. If the rubber_duck is not visible: If all obstacles are at distance > 5, initiate target search (output 'left' or 'right')\n"
                "Output must be a single JSON object with **two keys only**:\n"
                "- 'object': list of all detected objects\n"
                "- 'command': one of ['forward', 'left', 'right', 'stop'] and do not include puncuation, explanation, or prefixes.\n"
                "Strictly output raw JSON only. No explanation, no code block, no extra text."
            )

            prompt_user = [
                {
                    "type": "text",
                    "text": "Analyze the image and perform both tasks."
                },
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/png;base64,{image_base64}"
                    }
                }
            ]

            # GPT-4o 호출
            response = openai.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {"role": "system", "content": prompt_system},
                    {"role": "user", "content": prompt_user}
                ],
                max_tokens=600,
                temperature=0.7
            )

            content = response.choices[0].message.content.strip()

            # 코드블록 제거
            if content.startswith("```json") or content.startswith("```"):
                content = re.sub(r"^```json\s*|\s*```$", "", content.strip())

            # JSON 파싱
            result = json.loads(content)
            objects = result.get("object", [])
            command = result.get("command", "")

            # 퍼블리시
            self.description_pub.publish(String(data=json.dumps({"object": objects}, ensure_ascii=False)))
            self.command_pub.publish(String(data=command))
            self.get_logger().info(f"[GPT 응답] objects: \n{json.dumps(objects, ensure_ascii=False, indent=2)}")
            self.get_logger().info(f"[GPT 응답] command: {command}")

            # 다음 이미지 저장 트리거
            self.trigger_pub.publish(String(data='next'))

        except Exception as e:
            self.get_logger().error(f"GPT 처리 중 오류 발생: {e}")
            self.trigger_pub.publish(String(data='next'))

def main(args=None):
    rclpy.init(args=args)
    node = GPTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
