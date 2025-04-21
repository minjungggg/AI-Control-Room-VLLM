import os
import base64
import threading
import time
import openai
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class GPTImageRobotController(Node):
    def __init__(self):
        super().__init__('gpt_image_robot_controller')

        openai.api_key = os.getenv("GPT_API_KEY")
        self.image_path = os.path.expanduser('~/saved_images/latest_image.png')

        self.move_pub = self.create_publisher(String, 'move_robot', 10)
        self.direction_pub = self.create_publisher(String, 'move_direction', 10)
        self.stop_pub = self.create_publisher(String, 'stop_robot', 10)
        self.thrust_busy_sub = self.create_subscription(Bool, 'thrust_busy', self.thrust_busy_callback, 10)

        self.thrust_is_busy = False
        self.processing = False
        self.timer = self.create_timer(5.0, self.timer_callback)

        self.get_logger().info("GPT Image Robot Controller Node Started")

    def thrust_busy_callback(self, msg: Bool):
        self.thrust_is_busy = msg.data

    def timer_callback(self):
        if self.processing:
            self.get_logger().debug("Currently processing, ignore new analysis requests")
            return

        # 🔒 thrust_is_busy 방어
        if self.thrust_is_busy:
            self.get_logger().info("THE ROBOT IS IN BUSY STATE, SO DO NOT RUN ANALYZE_IMAGE.")
            return

        self.processing = True
        threading.Thread(target=self.analyze_and_act, daemon=True).start()

    def analyze_and_act(self):
        try:
            if self.thrust_is_busy:
                self.get_logger().info("STOP ANALYSIS BECAUSE THE ROBOT IS BUSY.")
                return

            if not os.path.exists(self.image_path):
                self.get_logger().warn(f"IMAGE FILE DOES NOT EXIST: {self.image_path}")
                return

            with open(self.image_path, "rb") as img_file:
                image_bytes = img_file.read()
                image_data = self._to_base64(image_bytes)

            # 1단계: GPT에게 이미지 설명 요청
            describe_response = openai.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "You are acting as a judge for the water drone control system on the voyage."
                            "Water drones are twin-shaped, and the black protrusions at the bottom of the image are engines on both sides of the drone, and this interval is the actual horizontal length of the drone. "
                            "The vertical length is about twice this length, and the camera is located 0.85m behind the front, this location is about one-third the vertical length of the drone. "
                            "Recognize the drone's size based on the above description. Once again, the gray object at the bottom of the center of the image is the engine part of the drone. It is not an obstacle."
                            "Analyze the image to determine the location of the buoy, the presence of obstacles, and then explain the direction of navigation by considering the size of the drone."
                            "It would be nice to listen to the explanation and be detailed enough for you to draw a similar picture."
                        )
                    },
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": "Please analyze the image and explain the surrounding situation."},
                            {"type": "image_url", "image_url": {"url": "data:image/png;base64," + image_data}}
                        ]
                    }
                ],
                max_tokens=300,
                temperature=0.5,
                top_p=0.5
            )

            description = describe_response.choices[0].message.content.strip()
            self.get_logger().info(f"[DESCRIPTION] {description}")

            # 2단계: GPT에게 stop 또는 move 판단 요청
            decision_response = openai.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "You are the navigation judgment system of a sailing drone. Based on the description, respond only to either 'stop' or 'move'. "
                            "Be sure to print out only one word without explanation."
                        )
                    },
                    {"role": "assistant", "content": description},
                    {
                        "role": "user",
                        "content": (
                            "I want to find a yellow duck and move drone's left engine to position it in front of the duck."
                            "If you haven't found the duck, print out the 'move' command to move, and find the duck as you move."
                        )
                    }
                ],
                max_tokens=10,
                temperature=0.2,
                top_p=0.2
            )

            decision = decision_response.choices[0].message.content.strip().lower()
            self.get_logger().info(f"[DECISION] {decision}")

            if decision not in ["stop", "move"]:
                self.get_logger().warn(f"UNEXPECTED DECISION: {decision}")
                return

            if decision == "stop":
                self.get_logger().info("COMMAND 'stop' ")
                self.stop_pub.publish(String(data="stop"))
                rclpy.shutdown()
                return

            # 3단계: move인 경우 방향 판단
            self.get_logger().info("COMMAND 'move' ")
            self.move_pub.publish(String(data="move"))

            direction_response = openai.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "You are the system for determining the direction of a sailing drone. Be sure to respond with only one of 'w', 'a', 's', and 'd'."
                            "Water drones are twin-shaped, and the black protrusions at the bottom of the image are engines on both sides of the drone, and this interval is the actual horizontal length of the drone. "
                            "The vertical length is about twice this length, and the camera is located 0.85m behind the front, this location is about one-third the vertical length of the drone. "
                            "Recognize the drone's size based on the above description. Once again, the gray object at the bottom of the center of the image is the engine part of the drone. It is not an obstacle."
                            "'w' = Front, 'a' = left turn, 's' = backward, 'd' = right turn. Print only one letter without explanation."
                        )
                    },
                    {"role": "assistant", "content": description},
                    {
                        "role": "user",
                        "content": (
                            "I want to find a yellow duck and move drone's left engine to position it in front of the duck."
                            "If there is no 'duck' in the description, let's move to avoid obstacles based on the description so we can find the duck."
                        )
                    }
                ],
                max_tokens=10,
                temperature=0.3,
                top_p=0.3
            )

            direction = direction_response.choices[0].message.content.strip().lower()
            if direction in ['w', 'a', 's', 'd']:
                self.direction_pub.publish(String(data=direction))
                self.get_logger().info(f"[DIRECTION] '{direction}' TOPIC PUB")
            else:
                self.get_logger().warn(f"UNEXPECTED DIRECTION: {direction}")

        except Exception as e:
            self.get_logger().error(f"[ERROR] {str(e)}")

        finally:
            self.processing = False

    def _to_base64(self, image_bytes):
        return base64.b64encode(image_bytes).decode('utf-8')


def main(args=None):
    rclpy.init(args=args)
    node = GPTImageRobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt >>> SHUTDOWN.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
