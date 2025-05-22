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
        # theta_deg (θ)
        y_offset = y - (image_height / 2)
        theta_deg = (y_offset / (image_height / 2)) * (fov_v / 2)
        theta_rad = math.radians(theta_deg)

        # phi_deg (φ)
        x_offset = x - (image_width / 2)
        phi_deg = (x_offset / (image_width / 2)) * (fov_h / 2)
        phi_rad = math.radians(phi_deg)

        # calculate distance
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
                            "      {\"name\": \"red buoy\", \"position\": \"left 15º\", \"distance\": 5 },\n"
                            "      {\"name\": \"black buoy\", \"position\": \"right 26º\", \"distance\": 8 }\n"
                            "    ],\n"
                            "    \"duck\": {\"found\": true, \"position\": \"left 5º\", \"distance\": \"unknown\" }\n"
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
                                    "some contours have uncertain classification due to overlap or shape irregularity"
                                    "If there is something wrong with the distance calculation or position, you can change the value by judging it arbitrarily."
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
        return description

    def parse_description(self, description_str):
        if description_str.startswith("```"):
            description_str = re.sub(r"```(json)?", "", description_str).strip()
            description_str = re.sub(r"```", "", description_str).strip()
        parsed = json.loads(description_str)
        desc = parsed["description"]
        return desc

    def get_duck_prev_position_note(self, previous_desc_json):
        if not previous_desc_json:
            return "The duck has not been seen previously."

        duck = previous_desc_json.get("duck", {})
        if duck.get("found"):
            pos = duck.get("position", "unknown")
            return f"In the previous frame, the duck was seen at position: {pos}."
        else:
            return "The duck was not found in the previous frame."

    def request_decision_and_direction(self, desc_str: str, duck_note: str):
        prompt = (
            "You are the navigation system of a sailing water drone.\n"
            "The drone is twin-hull (catamaran-style), 2.5m wide, 5m long, and 1.5m high.\n"
            "The camera is mounted 0.85 meters from the front and 1.1 meters above the water surface.\n\n"
            f"{duck_note}\n\n"
            "Use the following rules:\n"
            "1. If there are no obstacles and no duck is visible, rotate ('a' or 'd') to search.\n"
            "2. If there are no obstacles and the duck is visible:\n"
            "    - If the duck is far (>10m), move forward ('w') to approach it.\n"
            "    - If the duck is close (≤10m), center the duck in the view and stop.\n"
            "3. If obstacles are far (>8m):\n"
            "    - If the duck is not visible, move forward or rotate freely to explore the area.\n"
            "    - If the duck is visible, move forward in a direction that keeps distance from the obstacles while approaching the duck.\n"
            "4. If obstacles are close (≤8m):\n"
            "    - If the duck is not visible, rotate away from the nearest obstacle to find the duck.\n"
            "    - If the duck is far (>10m), move forward only in a direction that turns away from the obstacle.\n"
            "    - If the duck is close (≤10m), first adjust the drone to keep away from the obstacle, then rotate or move to center the duck.\n"
            "5. If the duck is centered and its distance is within 2 meters, stop.\n\n"
            "Respond strictly in the following JSON format:\n"
            "{\n"
            "  \"decision\": \"move\" or \"stop\",\n"
            "  \"direction\": \"w\" or \"a\" or \"s\" or \"d\"\n"
            "}"
        )

        response = openai.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": prompt},
                {"role": "user", "content": desc_str}
            ],
            max_tokens=100,
            temperature=0.5,
            top_p=0.8
        )

        result_text = response.choices[0].message.content.strip()

        try:
            # Strip code block formatting if present
            if result_text.startswith("```"):
                result_text = re.sub(r"```(json)?", "", result_text).strip()
                result_text = re.sub(r"```", "", result_text).strip()

            result = json.loads(result_text)
            decision = result.get("decision", "").lower()
            direction = result.get("direction", "").lower()

            self.get_logger().info(f"[COMBINED DECISION] decision: {decision}, direction: {direction}")
            return decision, direction

        except Exception as e:
            self.get_logger().error(f"[ERROR parsing decision+direction] {e} | Raw: {result_text}")
            return None, None


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
            
            center_width = int(image_width * 0.3)
            side_width = (image_width - center_width) // 2

            cv2.rectangle(general_color_mask, (0, bottom_ignore_y), (side_width, image_height), 0, -1)
            cv2.rectangle(general_color_mask, (image_width - side_width, bottom_ignore_y), (image_width, image_height), 0, -1)

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
            desc_str = json.dumps(desc_json, indent=2)

            duck_note = self.get_duck_prev_position_note(getattr(self, 'last_desc_json', None))

            decision, direction = self.request_decision_and_direction(desc_str, duck_note)

            if decision == "stop":
                self.stop_pub.publish(String(data="stop"))
            elif decision == "move" and direction in ['w', 'a', 's', 'd']:
                self.direction_pub.publish(String(data=direction))
            else:
                self.get_logger().warn("[WARNING] Invalid decision/direction from GPT.")
            self.last_desc_json = desc_json
            
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
