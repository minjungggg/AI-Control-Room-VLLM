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

        if self.thrust_is_busy:
            self.get_logger().info("THE ROBOT IS IN BUSY STATE, SO DO NOT RUN ANALYZE_IMAGE.")
            return

        self.processing = True
        threading.Thread(target=self.main_process, daemon=True).start()

    def get_latest_image_path(self):
        image_dir = os.path.expanduser('~/saved_images')
        pattern = re.compile(r'saved_image_(\d+)\.png')

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

    @staticmethod
    def estimate_corrected_distance(x, y, image_width, image_height, fov_h=90.0, fov_v=60.0, camera_height=1.1):
        # 수직 각도 (theta): 화면 중앙 기준
        y_offset = y - (image_height / 2)
        theta_deg = (y_offset / (image_height / 2)) * (fov_v / 2)
        theta_rad = math.radians(theta_deg)

        # 수평 각도 (phi): 화면 중앙 기준
        x_offset = x - (image_width / 2)
        phi_deg = (x_offset / (image_width / 2)) * (fov_h / 2)
        phi_rad = math.radians(phi_deg)

        # 거리 계산
        if abs(math.tan(theta_rad)) < 1e-6:
            return float('inf')  # 혹은 999.0, max distance 등 처리

        depth_z = camera_height / math.tan(theta_rad)
        corrected_distance = depth_z / math.cos(phi_rad)

        return round(corrected_distance, 2)


    
    def request_gpt_description(self, image_data, image_path, contour_json):
        self.get_logger().info(f"*****************************{image_path}*****************************")
        response = openai.chat.completions.create(
            model="gpt-4o",
            messages=[
                    {
                        "role": "system",
                        "content": (
                            "You are acting as a reasoning engine for a water drone control system. "
                            "The drone is twin-hull (catamaran-style). The gray objects at the bottom center of the image are engines attached to both sides of the drone—do not classify them as obstacles. "
                            "The drone is 2.5m wide, 5m long, and 1.5m high. The camera is mounted 0.85 meters from the front and 1.1 meters above the water surface. "
                            "All visible objects (e.g., buoys, ducks, barrages) should be identified with name, position, and estimated distance. "
                            "You are given additional image contour analysis, which includes: "
                            "- id: unique contour identifier\n"
                            "- bottom_pixel: pixel coordinate where object touches water\n"
                            "- distance_m: estimated distance in meters\n"
                            "- area: size of the contour in pixels\n"
                            "- aspect_ratio: height divided by width\n\n"
                            "Use this data to infer what each object might be. Match contour characteristics with what is seen in the image. "
                            "DO NOT include the contour_analysis block in your output. Instead, convert it into a structured JSON like the example below.\n"
                            "Respond ONLY with a single JSON block in this structure:\n\n"
                            "{\n"
                            "  \"description\": {\n"
                            "    \"obstacles\": [\n"
                            "      {\"name\": \"red buoy\", \"position\": \"front\", \"distance\": 5 },\n"
                            "      {\"name\": \"black buoy\", \"position\": \"left\", \"distance\": 10 }\n"
                            "    ],\n"
                            "    \"duck\": {\"found\": true, \"position\": \"front-right\", \"distance\": \"unknown\" }\n"
                            "  }\n"
                            "}"
                        )
                    },
                    {
                        "role": "user",
                        "content": [
                            {"type": "image_url", "image_url": {"url": "data:image/png;base64," + image_data}},
                            {
                                "type": "text",
                                "text": (
                                    "The image below shows water obstacles captured by the drone. "
                                    "Here is additional analysis from image contours with their id, bottom pixel, distance, area, and aspect ratio:\n"
                                    f"{contour_json}\n\n"
                                    "Using this information, infer what each object might represent and construct the final JSON response accordingly. "
                                    "Do not include the original contour list in your response."
                                )
                            }
                        ]
                    }
                ],
            max_tokens=400,
            temperature=0.8,
            top_p=0.6
        )
        description = response.choices[0].message.content.strip()
        self.get_logger().info(f"[DESCRIPTION] {description}")
        return description

    def parse_description(self, description_str):
        if description_str.startswith("```"):
            description_str = re.sub(r"```(json)?", "", description_str).strip()
            description_str = re.sub(r"```", "", description_str).strip()
        parsed = json.loads(description_str)
        desc = parsed["description"]
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
                        "- If there is an obstacle in front within 0.8m or closer, respond with 's' to move backward."
                        "- If there is an obstacle on the left or right within 0.5m or closer, respond with 's' to move backward."
                        "- If no such threat exists, but obstacles are detected, steer ('a' or 'd') based on the safer direction."
                        "- If no obstacles are close, and the duck has not been found, move to closer or find duck by rotating"
                        "- If duck is found and it is not centered, adjust direction ('a' or 'd') to center it."
                        "- Respond with exactly one of: 'w', 'a', 's', 'd'. Do not explain."
                    )

                },
                {"role": "assistant", "content": full_description},
                {
                    "role": "user",
                    "content": (
                        "I want to move the yellow duck between the grey drone engines shown in the picture to the center, bottom of the picture."
                        "If the 'found' of 'duck' is 'false' in the description, let's avoid obstacles and look for ducks."
                        "If there is no risk of hitting an obstacle, it is recommended that move closer to the obstacle "
                    )
                }
            ],
            max_tokens=10,
            top_p=0.3
        )
        direction = response.choices[0].message.content.strip().lower()
        self.get_logger().info(f"[DIRECTION] {direction}")
        return direction

    def main_process(self):
        try:
            image_path = self.get_latest_image_path()
            if not image_path or not os.path.exists(image_path):
                self.get_logger().warn("Required image file not found.")
                self.processing = False
                return

            image_data = self.image_to_base64(image_path)
            image = cv2.imread(image_path)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)
            contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            image_height, image_width = gray.shape
            valid_contours = [c for c in contours if 60 <= cv2.contourArea(c) <= 600]

            contour_summary = []
            for idx, contour in enumerate(valid_contours):
                area = cv2.contourArea(contour)
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = round(h / w, 2) if w != 0 else 0
                bottom = max(contour, key=lambda p: p[0][1])[0]
                x_b, y_b = bottom
                distance = self.estimate_corrected_distance(x_b, y_b, image_width, image_height)
                
                contour_summary.append({
                    "id": idx + 1,
                    "bottom_pixel": [int(x_b), int(y_b)],
                    "distance_m": distance,
                    "area": round(area, 1),
                    "aspect_ratio": aspect_ratio
                })

            contour_json = json.dumps({"contour_analysis": contour_summary}, indent=2)
            self.get_logger().info(f"[CONTOUR JSON] {contour_json}")
            description_str = self.request_gpt_description(image_data, image_path, contour_json)
            desc_json = self.parse_description(description_str)
            desc_str = json.dumps(desc_json)

            decision = self.request_decision(desc_str, self.latest_velocity_data or "")
            if decision == "stop":
                self.stop_pub.publish(String(data="stop"))
                rclpy.shutdown()
                return
            elif decision == "move":
                self.move_pub.publish(String(data="move"))
                direction = self.request_direction(desc_str)
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
