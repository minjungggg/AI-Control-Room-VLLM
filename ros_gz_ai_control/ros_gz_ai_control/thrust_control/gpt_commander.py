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
        # 수직 각도 (θ)
        y_offset = y - (image_height / 2)
        theta_deg = (y_offset / (image_height / 2)) * (fov_v / 2)
        theta_rad = math.radians(theta_deg)

        # 수평 각도 (φ)
        x_offset = x - (image_width / 2)
        phi_deg = (x_offset / (image_width / 2)) * (fov_h / 2)
        phi_rad = math.radians(phi_deg)

        # 거리 계산
        if abs(math.tan(theta_rad)) < 1e-6:
            distance = float('inf')
        else:
            depth_z = camera_height / math.tan(theta_rad)
            distance = depth_z / math.cos(phi_rad)

        return round(distance, 2), round(phi_deg, 2)

    
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
                            "      {\"name\": \"red buoy\", \"position\": \"left 15deg\", \"distance\": 5 },\n"
                            "      {\"name\": \"black buoy\", \"position\": \"right 26deg\", \"distance\": 8 }\n"
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
                                    "Here is additional analysis from image contours with their id, bottom pixel, distance, area, and horizontal_angle:\n"
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
                        "I want to find a yellow duck, and place it close front of drone engines."
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

    def request_multiple_directions(self, full_description, n=3):
        directions = []

        for i in range(n):
            response = openai.chat.completions.create(
                model="gpt-4o",
                messages=[
                    {
                        "role": "system",
                        "content": (
                            "You are the system for determining the direction of a sailing drone. Respond with ONLY ONE of: 'w', 'a', 's', or 'd'."
                            "The water drone is twin-hull (catamaran-style)."
                            "Use the following rules to decide:"
                            "- 'w' to move forward, 'a' to turn left, 'd' to turn right, and 's' to move backward."
                            "- If there is an obstacle in front within 0.8m or closer, respond with 's' to move backward."
                            "- If there is an obstacle on the left or right within 0.5m or closer, respond with 's'."
                            "- If no such threat exists, but obstacles are detected, steer ('a' or 'd') based on the safer direction."
                            "- If no obstacles are close and the duck has not been found, move to search."
                            "- If duck is found and off-centered, adjust with 'a' or 'd'."
                            "- Respond with exactly one of: 'w', 'a', 's', or 'd'. Do not explain."
                        )
                    },
                    {"role": "assistant", "content": full_description},
                    {
                        "role": "user",
                        "content": (
                            "I want to find a yellow duck, and place it close front of drone engines."
                            "If 'duck' is not found, try to avoid obstacles and rotate to look for it."
                        )
                    }
                ],
                max_tokens=10,
                temperature=0.7,  # 다양성 확보
                top_p=0.9
            )
            direction = response.choices[0].message.content.strip().lower()
            directions.append(direction)

        return directions

    def request_direction_evaluation(self, full_description, directions):
        # 여러 방향 정리
        direction_lines = "\n".join([f"{i+1}. '{d}'" for i, d in enumerate(directions)])

        compare_prompt = (
            f"The following navigation decisions were made by the system based on the same description:\n\n"
            f"{full_description}\n\n"
            f"Here are the direction outputs:\n{direction_lines}\n\n"
            f"Evaluate them and choose the best direction.\n"
            f"Respond with ONLY ONE of: 'w', 'a', 's', or 'd'.\n"
            f"Do not explain. Just respond with the final choice."
        )

        response = openai.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": "You are an evaluator choosing the best navigation direction for a sailing drone."
                            "The water drone is twin-hull (catamaran-style)."
                            "Use the following rules to decide:"
                            "- 'w' to move forward, 'a' to turn left, 'd' to turn right, and 's' to move backward."
                            "- If there is an obstacle in front within 0.8m or closer, respond with 's' to move backward."
                            "- If there is an obstacle on the left or right within 0.5m or closer, respond with 's'."
                            "- If no such threat exists, but obstacles are detected, steer ('a' or 'd') based on the safer direction."
                            "- If no obstacles are close and the duck has not been found, move to search."
                            "- If obstacles and duck are far from the drone (over than 10), move forward ('w')."
                            "- If duck is found and off-centered, adjust with 'a' or 'd'."
                            "- Respond with exactly one of: 'w', 'a', 's', or 'd'. Do not explain."},
                {"role": "user", "content": compare_prompt}
            ],
            temperature=0.3,
            max_tokens=5
        )

        final_choice = response.choices[0].message.content.strip().lower()
        self.get_logger().info(f"[FINAL CHOICE] {final_choice}")
        return final_choice


    def main_process(self):
        try:
            image_path = self.get_latest_image_path()
            if not image_path or not os.path.exists(image_path):
                self.get_logger().warn("Required image file not found.")
                self.processing = False
                return

            image_data = self.image_to_base64(image_path)
            image = cv2.imread(image_path)
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            image_height, image_width = image.shape[:2]

            # ✅ HSV color range for object detection
            hsv_ranges = [
                ((0, 50, 50), (10, 255, 255)),       # red1 
                ((160, 50, 50), (180, 255, 255)),    # red2
                ((20, 50, 50), (40, 255, 255)),      # yellow
                ((0, 0, 0), (180, 255, 80)),         # black 
                ((130, 30, 30), (160, 255, 255)),    # purple
                ((35, 50, 50), (85, 255, 255)),      # green 
            ]

            masks = [cv2.inRange(hsv, lower, upper) for (lower, upper) in hsv_ranges]
            general_color_mask = masks[0]
            for m in masks[1:]:
                general_color_mask = cv2.bitwise_or(general_color_mask, m)

            # ✅ Masking (erase top-sky and bottom-engines)
            top_ignore_y = int(image_height * 0.2)      
            bottom_ignore_y = int(image_height * 0.9)   
            cv2.rectangle(general_color_mask, (0, 0), (image_width, top_ignore_y), 0, -1)
            cv2.rectangle(general_color_mask, (0, bottom_ignore_y), (image_width, image_height), 0, -1)

            contours, _ = cv2.findContours(general_color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # # ✅ Visulaization (for debugging)
            # contour_vis = image.copy()
            # cv2.drawContours(contour_vis, contours, -1, (0, 255, 255), 2)
            # visualization_path = os.path.expanduser("~/saved_images/contour_visualization.png")
            # cv2.imwrite(visualization_path, contour_vis)
            # self.get_logger().info(f"[DEBUG] Contour visualization saved at: {visualization_path}")

            # ✅ JSON 
            contour_summary = []
            for idx, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = round(h / w, 2) if w != 0 else 0
                bottom = max(contour, key=lambda p: p[0][1])[0]
                x_b, y_b = bottom
                distance, horizontal_angle = self.estimate_corrected_distance(x_b, y_b, image_width, image_height)

                contour_summary.append({
                    "id": idx + 1,
                    "bottom_pixel": [int(x_b), int(y_b)],
                    "distance_m": distance,
                    "horizontal_angle": horizontal_angle,
                    "area": round(area, 1),
                    "aspect_ratio": aspect_ratio
                })

            contour_json = json.dumps({"contour_analysis": contour_summary}, indent=2)
            # self.get_logger().info(f"[CONTOUR JSON] {contour_json}")

            description_str = self.request_gpt_description(image_data, image_path, contour_json)
            desc_json = self.parse_description(description_str)
            desc_str = json.dumps(desc_json)

            decision = self.request_decision(desc_str, self.latest_velocity_data or "")
            if decision == "stop":
                self.stop_pub.publish(String(data="stop"))
                rclpy.shutdown()
                return
            elif decision == "move":
                directions = self.request_multiple_directions(desc_str, n=3)
                direction = self.request_direction_evaluation(desc_str, directions)
                if direction in ['w', 'a', 's', 'd']:
                    self.direction_pub.publish(String(data=direction))
                else:
                    self.get_logger().warn(f"[WARNING] Invalid direction received from evaluator: {direction}")


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
