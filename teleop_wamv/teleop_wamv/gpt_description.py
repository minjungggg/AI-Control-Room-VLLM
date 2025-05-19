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
                            "The horizontal field of view (FOV) of the camera is 80 degrees. "
                            "Ignore any UI elements visible in the image (e.g., green horizontal lines, blue vertical lines) and the two gray objects at the bottom of the screen, which are the drone’s engines. "
                            "Summarize **all major objects** appearing in the image in JSON format. "
                            "Each object must include the following properties: type, color, angle, distance. "
                            "- type: Must be either obstacle or rubber_duck. "
                            "- angle: Relative to the drone’s center; 0 degrees is straight ahead, negative values indicate left, and positive values indicate right (unit: degrees). "
                            "- distance: Estimate the distance from the drone to the object using an integer between 0 and 10. "
                            "The value should be based solely on how far the object is from the drone, not on the distance between multiple objects."
                            "A larger object appearing low in the image is closer (e.g., 0–3), while a smaller object higher in the image is farther (e.g., 7–10)."
                            "The distance scale must be applied consistently across all images and must reflect the physical distance from the drone to each object, not their spacing. "
                            "An obstacle is any object other than the target. "
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
                temperature=0.6,
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