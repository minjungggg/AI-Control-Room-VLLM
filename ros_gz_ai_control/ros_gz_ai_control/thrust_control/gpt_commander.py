import os
import re
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

    def get_latest_image_path(self):
        image_dir = os.path.expanduser('~/saved_images')
        pattern = re.compile(r'bgra8_saved_image_(\d+)\.png')

        try:
            files = os.listdir(image_dir)
            numbered_files = []
            for f in files:
                match = pattern.fullmatch(f)
                if match:
                    index = int(match.group(1))
                    numbered_files.append((index, f))

            if not numbered_files:
                return None

            latest_file = max(numbered_files, key=lambda x: x[0])[1]
            return os.path.join(image_dir, latest_file)
        except Exception as e:
            self.get_logger().error(f"WE CAN'T FIND LATEST IMAGE: {e}")
            return None

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

            latest_image_path = self.get_latest_image_path()
            if not latest_image_path or not os.path.exists(latest_image_path):
                self.get_logger().warn("LATEST_IMAGE FILE DOES NOT EXIST")
                return

            with open(latest_image_path, "rb") as img_file:
                image_bytes = img_file.read()
                image_data = self._to_base64(image_bytes)
                self.get_logger().info(f"***************[FILE] {os.path.basename(latest_image_path)}***************")

            # request GPT to analyze the image
            describe_response = openai.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "You are acting as a judge for the water drone control system on the voyage. "
                            "The water drone is twin-hull (catamaran-style). The gray objects at the bottom center of the image are engines, attached to both sides of the drone—not obstacles. "
                            "The horizontal width between these engines represents the actual width of the drone. The vertical length is about twice this length. "
                            "The camera is mounted 0.85 meters from the front of the drone. This is roughly one-third from the front toward the back of the drone. "
                            "Gray engine parts are part of the drone, not obstacles. All other visible objects (e.g., buoys or ducks) should be analyzed for position and proximity. "
                            "Use the drone's known dimensions as a reference to estimate distances. If an object is more than 1 meter away, it is not a threat. Closer than 1 meter? That may require avoidance. "
                            "Analyze the image and output the result in the following JSON format only: "
                            "{"
                            "  \"obstacles\": ["
                            "    {\"name\": \"red buoy\", \"position\": \"front\", \"distance\": 1.5, \"threat\": false},"
                            "    {\"name\": \"black buoy\", \"position\": \"front-left\", \"distance\": 0.8, \"threat\": true}"
                            "  ],"
                            "  \"duck\": {\"found\": true, \"position\": \"front-right\", \"distance\": 0.6},"
                            "  \"any_threat\": true"
                            "}"
                        )

                    },
                    {
                        "role": "user",
                        "content": [
                            {"type": "image_url", "image_url": {"url": "data:image/png;base64," + image_data}},
                            {"type": "text", "text": "Please analyze the image and explain the surrounding situation."}   
                        ]
                    }
                ],
                max_tokens=300,
                temperature=0.5,
                top_p=0.5
            )

            description = describe_response.choices[0].message.content.strip()
            self.get_logger().info(f"[DESCRIPTION] {description}")

            # request GPT to make a decision 'stop' or 'move'
            decision_response = openai.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "You are the navigation judgment system of a sailing drone. Based on the description, respond only to either 'stop' or 'move'. "
                            "If an object (like a buoy) is more than 1 meter away, it is not a threat—no avoidance action is needed. Closer than 1 meter? That may require movement or avoidance."
                            "Be sure to print out only one word without explanation."
                        )
                    },
                    {"role": "assistant", "content": description},
                    {
                        "role": "user",
                        "content": (
                            "I want to find a yellow duck, and place it close between the grey drone engines shown in the picture."
                            "If you haven't found a yellow duck, print out a 'move' command to move and look for a yellow duck on the move."
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

            # if move decision
            self.get_logger().info("COMMAND 'move' ")
            self.move_pub.publish(String(data="move"))

            direction_response = openai.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "You are the system for determining the direction of a sailing drone. Be sure to respond with only one of 'w', 'a', 's', and 'd'."
                            "The water drone is twin-hull (catamaran-style). The gray objects at the bottom center of the image are engines, attached to both sides of the drone—not obstacles."
                            "The horizontal width between these engines represents the actual width of the drone. The vertical length is about twice this length"
                            "The camera is mounted 0.85 meters from the front of the drone. This is roughly one-third from the front toward the back of the drone."
                            "Gray engine parts are part of the drone, not obstacles. All other visible objects (e.g., buoys) should be analyzed for position and proximity. Recognize the drone's size based on the above description."
                            "If an object (like a buoy) is more than 1 meter away, it is not a threat—no avoidance action is needed. Closer than 1 meter? That may require avoidance."
                            "'w' = Front, 'a' = left turn, 's' = backward, 'd' = right turn. Print only one letter without explanation."
                        )
                    },
                    {"role": "assistant", "content": description},
                    {
                        "role": "user",
                        "content": (
                            "I want to find a yellow duck, and place it close between the grey drone engines shown in the picture."
                            "If there is no 'duck' in the description, let's move to avoid obstacles based on the description so we can find the duck."
                            "If there is no risk of hitting an obstacle, it is recommended that move closer to the obstacle "
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
