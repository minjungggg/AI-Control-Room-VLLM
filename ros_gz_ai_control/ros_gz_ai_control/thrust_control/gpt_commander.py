import os
import re
import base64
import threading
import json
import cv2
import math
import numpy as np
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
        self.velocity_json_sub = self.create_subscription(String, '/velocity_json', self.velocity_callback, 10)

        self.latest_velocity_data = None
        self.thrust_is_busy = False
        self.processing = False
        self.timer = self.create_timer(5.0, self.timer_callback)

        self.get_logger().info("GPT Image Robot Controller Node Started")

    def thrust_busy_callback(self, msg: Bool):
        self.thrust_is_busy = msg.data

    def velocity_callback(self, msg: String):
        self.latest_velocity_data = msg.data

    def timer_callback(self):
        if self.processing:
            self.get_logger().debug("Currently processing, ignore new analysis requests")
            return

        # 🔒 thrust_is_busy
        if self.thrust_is_busy:
            self.get_logger().info("THE ROBOT IS IN BUSY STATE, SO DO NOT RUN ANALYZE_IMAGE.")
            return

        self.processing = True
        threading.Thread(target=self.main_process, daemon=True).start()

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

    def image_to_base64(self, image_path):
        with open(image_path, "rb") as img_file:
            return base64.b64encode(img_file.read()).decode('utf-8')
    
    def base64_to_cv2(self, base64_str):
        img_bytes = base64.b64decode(base64_str)
        np_arr = np.frombuffer(img_bytes, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return img

    def request_gpt_description(self, image_data, image_path):
        self.get_logger().info(f"*****************************{image_path}*****************************")
        velocity_note = f"Current velocity: {self.latest_velocity_data}" if self.latest_velocity_data else "Velocity data not available."
        response = openai.chat.completions.create(
            model="gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "You are acting as a judge for the water drone control system on the voyage. "
                            "The water drone is twin-hull (catamaran-style). The gray objects at the bottom center of the image are engines, attached to both sides of the drone—not obstacles. "
                            "The drone's horizontal width is 2.5m, length is 5m, and height is 1.5m. "
                            "The camera is mounted 0.85 meters from the front of the drone and 1.1m above the water surface. "
                            "All other visible objects (e.g., buoys, ducks, barrages) should be analyzed for position and color. And defines HSV color ranges for each object color found in the image."
                            "Respond ONLY with a single JSON block with the following structure:"
                            "\n\n"
                            "{\n"
                            "  \"description\": {\n"
                            "    \"obstacles\": [\n"
                            "      {\"name\": \"red buoy\", \"position\": \"front\", \"color\": \"red\"},\n"
                            "      {\"name\": \"black buoy\", \"position\": \"left\", \"color\": \"black\"}\n"
                            "    ],\n"
                            "    \"duck\": {\"found\": true, \"position\": \"front-right\", \"color\": \"yellow\"}\n"
                            "  },\n"
                            "  \"color_hsv_dict\": {\n"
                            "    \"red\": [[0, 100, 100], [10, 255, 255]],\n"
                            "    \"black\": [[0, 0, 0], [180, 255, 50]],\n"
                            "    \"yellow\": [[20, 100, 100], [35, 255, 255]]\n"
                            "  }\n"
                            "}"
                        )
                    },
                    {
                        "role": "user",
                        "content": [
                            {"type": "image_url", "image_url": {"url": "data:image/png;base64," + image_data}},
                            {"type": "text", "text": "Please analyze the image and return the JSON as instructed."}
                        ]
                    }
                ],
                max_tokens=400,
                temperature=0.2,
                top_p=0.2
            )
        return response.choices[0].message.content.strip()

    def parse_description(self, description_str):
        if description_str.startswith("```"):
            description_str = re.sub(r"```(json)?", "", description_str).strip()
            description_str = re.sub(r"```", "", description_str).strip()
        parsed = json.loads(description_str)
        desc = parsed["description"]
        hsv_dict = {k: (tuple(v[0]), tuple(v[1])) for k, v in parsed["color_hsv_dict"].items()}
        self.get_logger().info(f"[DESCRIPTION]\n{desc}")
        self.get_logger().info(f"[HSV Dictionary]\n{hsv_dict}")
        return desc, hsv_dict

    def find_bottom_pixel(self, image, lower_hsv, upper_hsv):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        return tuple(largest[largest[:, :, 1].argmax()][0])  # (x, y)

    def estimate_distance(self, x, y, width=1920, height=1080, hfov=90.0, vfov=60.0, camera_height=1.1):
        center_x = width / 2
        center_y = height / 2

        # 각도를 라디안 단위로 변환
        hfov_rad = math.radians(hfov)
        vfov_rad = math.radians(vfov)

        # 화면 중심 기준으로 x, y 각도 계산
        theta_x = hfov_rad * ((x - center_x) / width)
        theta_y = vfov_rad * ((y - center_y) / height)

        # 수직 각도가 너무 작으면 계산 불가 (기울기 무한)
        if abs(theta_y) < 1e-3:
            return None

        # 수직 거리 (z축 기준)
        z = camera_height / math.tan(theta_y)

        # 수평 보정된 직선 거리
        distance = z / math.cos(theta_x)

        return round(distance, 2) if distance > 0 and math.isfinite(distance) else "unreachable"


    def calculate_object_distances(self, desc, hsv_dict, image):
        obstacles = desc.get("obstacles", [])
        duck = desc.get("duck") if isinstance(desc.get("duck"), dict) else None

        # 거리 계산 대상: buoy + barrage
        for obj in obstacles:
            color = obj.get("color", "").lower()
            if color not in hsv_dict:
                obj["distance"] = "unknown"
                continue

            bottom = self.find_bottom_pixel(image, *hsv_dict[color])
            if bottom is None:
                obj["distance"] = "unknown"
                continue

            x, y = bottom
            distance = self.estimate_distance(x, y)
            obj["distance"] = round(distance, 2) if isinstance(distance, (float, int)) and math.isfinite(distance) and distance > 0 else "unreachable"
            self.get_logger().info(f"[DEBUG] {obj['name']}: bottom=({x},{y}), distance={obj['distance']}")

        # duck은 따로 처리해서 덮어쓰기
        if duck:
            color = duck.get("color", "").lower()
            if color in hsv_dict:
                bottom = self.find_bottom_pixel(image, *hsv_dict[color])
                if bottom:
                    x, y = bottom
                    distance = self.estimate_distance(x, y)
                    duck["distance"] = round(distance, 2) if isinstance(distance, (float, int)) and math.isfinite(distance) and distance > 0 else "unreachable"
                    self.get_logger().info(f"[DEBUG] duck: bottom=({x},{y}), distance={duck['distance']}")
                else:
                    duck["distance"] = "unknown"
            else:
                duck["distance"] = "unknown"

        return desc



    def request_decision(self, full_description, velocity_note):
        response = openai.chat.completions.create(
            model="gpt-4o",
            messages=[
                {
                    "role": "system",
                    "content": (
                        "You are the navigation judgment system of a sailing drone. Based on the description, respond only to either 'stop' or 'move'. "
                        "The water drone is twin-hull (catamaran-style). The gray objects at the bottom center of the image are engines, attached to both sides of the drone—not obstacles. "
                        "The drone's width from left-engine to right-engine is 2.5m , The vertical length from front to behind is 5m , The height is 1.5m "
                        "The camera is mounted 0.85 meters from the front of the drone. and 1.1m from the water surface"
                        f"Additionally, drone's moving state here: {velocity_note}"
                        "Gray engine parts are part of the drone, not obstacles."
                        "Assess the threat level and distance to the obstacle by assuming the drone state after one second, taking into account its current speed and the fact that it is on the water."
                        "If an object (like a buoy) is more than 1 meter away, it is not a threat—no avoidance action is needed. Closer than 1 meter? That may require movement or avoidance."
                        "Be sure to print out only one word without explanation."
                    )
                },
                {"role": "assistant", "content": full_description},
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
        decision = response.choices[0].message.content.strip().lower()
        self.get_logger().info(f"[DECISION] {decision}")
        return decision

    def request_direction(self, full_description):
        response = openai.chat.completions.create(
            model="gpt-4o",
            messages=[
                {
                    "role": "system",
                    "content": (
                        "You are the system for determining the direction of a sailing drone. Respond with ONLY ONE of: 'w', 'a', 's', or 'd'."
                        "The water drone is twin-hull (catamaran-style)."
                        "Use the following rules to decide:"
                        "- If there is an obstacle **in front** within **0.8m or closer**, respond with 's' to move backward."
                        "- If there is an obstacle on the **left or right** within **0.5m or closer**, respond with 's' to move backward."
                        "- If no such threat exists, but obstacles are detected, steer ('a' or 'd') based on the safer direction."
                        "- If no obstacles are close, and the duck has **not been found**, move forward ('w') to search."
                        "- If duck is found and it is **not centered**, adjust direction ('a' or 'd') to center it."
                        "- Respond with exactly one of: 'w', 'a', 's', 'd'. Do not explain."
                    )

                },
                {"role": "assistant", "content": full_description},
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
        direction = response.choices[0].message.content.strip().lower()
        self.get_logger().info(f"[DIRECTION] {direction}")
        return direction

    def main_process(self):
        try:
            image_path = self.get_latest_image_path()
            if not image_path:
                self.get_logger().warn("No image found.")
                self.processing = False
                return

            image_data = self.image_to_base64(image_path)
            description_str = self.request_gpt_description(image_data, image_path)
            desc_json, hsv_dict = self.parse_description(description_str)
            image_cv = self.base64_to_cv2(image_data)
            updated_desc = self.calculate_object_distances(desc_json, hsv_dict, image_cv)

            self.get_logger().info(f"[UPDATED DESCRIPTION]\n{json.dumps(updated_desc, indent=2)}")

            updated_desc_str = json.dumps(updated_desc, indent=2)
            decision = self.request_decision(updated_desc_str, self.latest_velocity_data or "")
            if decision == "stop":
                self.stop_pub.publish(String(data="stop"))
                rclpy.shutdown()
                return
            elif decision == "move":
                self.move_pub.publish(String(data="move"))
                direction = self.request_direction(updated_desc_str)
                if direction in ['w', 'a', 's', 'd']:
                    self.direction_pub.publish(String(data=direction))

        except Exception as e:
            self.get_logger().error(f"[ERROR] {str(e)}")

        finally:
            self.processing = False


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
