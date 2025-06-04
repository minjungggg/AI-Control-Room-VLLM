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
                temperature=0.9,
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