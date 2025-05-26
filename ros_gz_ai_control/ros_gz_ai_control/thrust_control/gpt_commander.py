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

        self.in_path_mode = False
        self.path_plan = []
        self.current_step = 0

        self.thrust_is_busy = False
        self.processing = False
        self.timer = self.create_timer(5.0, self.timer_callback)

    def thrust_busy_callback(self, msg: Bool):
        self.thrust_is_busy = msg.data

    def timer_callback(self):
        if self.processing or self.thrust_is_busy:
            return
        self.processing = True
        threading.Thread(target=self.main_process, daemon=True).start()

    def get_latest_image_path(self):
        image_dir = os.path.expanduser('~/saved_images')
        pattern = re.compile(r'saved_image_(\d+)\.png')
        try:
            files = os.listdir(image_dir)
            numbered_files = [(int(match.group(1)), f) for f in files if (match := pattern.fullmatch(f))]
            if not numbered_files:
                return None
            latest_file = max(numbered_files)[1]
            return os.path.join(image_dir, latest_file)
        except:
            return None

    def image_to_base64(self, image_path):
        with open(image_path, "rb") as img_file:
            return base64.b64encode(img_file.read()).decode('utf-8')

    @staticmethod
    def estimate_corrected_distance(x, y, image_width, image_height, fov_h=90.0, fov_v=60.0, camera_height=1.1):
        y_offset = y - (image_height / 2)
        theta_deg = (y_offset / (image_height / 2)) * (fov_v / 2)
        theta_rad = math.radians(theta_deg)
        x_offset = x - (image_width / 2)
        phi_deg = (x_offset / (image_width / 2)) * (fov_h / 2)
        phi_rad = math.radians(phi_deg)
        if abs(math.tan(theta_rad)) < 1e-6:
            return float('inf'), round(phi_deg, 2)
        depth_z = camera_height / math.tan(theta_rad)
        distance = depth_z / math.cos(phi_rad)
        return round(distance, 2), round(phi_deg, 2)

    def request_threat_assessment_from_image(self, image_data, image_path, contour_json):
        prompt = (
            "You are the navigation system of an autonomous water drone.\n"
            "The drone is twin-hull (catamaran-style), 2.5m wide, 5m long, and 1.5m high.\n"
            "The camera is mounted 0.85 meters from the front and 1.1 meters above the water surface.\n\n"
            "Your only task is to decide whether the drone should STOP or continue MOVE, based on obstacles and duck position.\n\n"
            "Use the following rules:\n"
            "1. If obstacles are close (≤8m), respond with \"stop\".\n"
            "2. If the duck is centered and within 2 meters, respond with \"stop\".\n"
            "3. Except for the above cases, respond with \"move\".\n\n"
            "Respond ONLY with the following JSON format — do NOT include any other fields such as direction or explanations:\n"
            "{\n"
            "  \"decision\": \"move\" or \"stop\"\n"
            "}"
        )

        self.get_logger().info(f"###############[THREAT CHECK] {image_path}###############")

        response = openai.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": prompt},
                {
                    "role": "user",
                    "content": [
                        {"type": "image_url", "image_url": {"url": "data:image/png;base64," + image_data}},
                        {"type": "text", "text": (
                            "Is it safe for the drone to continue moving forward?\n"
                            "Evaluate based only on this image and contour data.\n"
                            f"Contour analysis:\n{contour_json}"
                        )}
                    ]
                }
            ],
            max_tokens=50,
            temperature=0.3,
            top_p=0.8
        )

        result_text = response.choices[0].message.content.strip()
        self.get_logger().info(f"[THREAT CHECK RESPONSE] {result_text}")

        try:
            if result_text.startswith("```"):
                result_text = re.sub(r"```(json)?", "", result_text).strip()
                result_text = re.sub(r"```", "", result_text).strip()
            result = json.loads(result_text)
            decision = result.get("decision", "").strip().lower()
            if decision not in ["move", "stop"]:
                self.get_logger().warn(f"[THREAT CHECK] Unexpected decision '{decision}', defaulting to 'stop'")
                return "stop"
            return decision
        except Exception as e:
            self.get_logger().warn(f"[THREAT CHECK ERROR] {e}")
            return "stop"  # fallback for safety


    def request_decision_and_direction_from_image(self, image_data, image_path, contour_json):
        self.get_logger().info(f"*****************************{image_path}*****************************")
        prompt = (
            "You are the navigation system of an autonomous water drone.\n"
            "The drone is twin-hull (catamaran-style), 2.5m wide, 5m long, and 1.5m high.\n"
            "The camera is mounted 0.85 meters from the front and 1.1 meters above the water surface.\n\n"
            "Your task is to decide the next movement direction based on image and image contours\n"
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
            "Do not include any explanations, markdown formatting, or code block markers like ```json. "
            "Output only the raw JSON object."
            "{\n"
            "  \"decision\": \"move\" or \"stop\",\n"
            "  \"direction\": \"w\" or \"a\" or \"s\" or \"d\"\n"
            "  \"duck_found\": true or false\n"
            "}"
        )

        response = openai.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": prompt},
                {
                    "role": "user",
                    "content": [
                        {"type": "image_url", "image_url": {"url": "data:image/png;base64," + image_data}},
                        {"type": "text", "text": "I want to get to the duck if it exists, while avoiding obstacles.\n"
                                            "Only identify a duck if it is clearly present in the image.\n"
                                            "Do not assume a duck is always there. "
                                            "Use the contour analysis and image contents to determine presence.\n"
                                            "Place the duck at the center-bottom of the image **only if found**.\n"
                                            f"Contour analysis:\n{contour_json}"}
                    ]
                }
            ],
            max_tokens=100,
            temperature=0.5,
            top_p=0.8
        )
        result_text = response.choices[0].message.content.strip()
        self.get_logger().info(f"GPT response: {result_text}")
        try:
            if result_text.startswith("```"):
                result_text = re.sub(r"```(json)?", "", result_text).strip()
                result_text = re.sub(r"```", "", result_text).strip()
            result = json.loads(result_text)
            return result.get("decision", ""), result.get("direction", ""), result.get("duck_found", False)
        except:
            return None, None, False

    def request_path_plan_from_image(self, image_data, image_path, contour_json):
        self.get_logger().info(f"[PATH PLAN] From {image_path}")
        prompt = (
            "You are the navigation planner for an autonomous water drone.\n"
            "The drone is twin-hull (catamaran-style), 2.5m wide, 5m long, and 1.5m high.\n"
            "The camera is mounted 0.85 meters from the front and 1.1 meters above the water surface.\n\n"
            "Given the current image and contour analysis, plan a safe path of 3 to 5 movement steps "
            "towards the duck while avoiding obstacles. Only use: 'w', 'a', 's', 'd'.\n"
            "avoiding means to not hit the obstacles, and not to get too close to them.\n"
            "Respond strictly in the following JSON format:\n"
            "{ \"path\": [\"a\", \"w\", \"w\"] }"
        )
        response = openai.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": prompt},
                {
                    "role": "user",
                    "content": [
                        {"type": "image_url", "image_url": {"url": "data:image/png;base64," + image_data}},
                        {"type": "text", "text": f"Contour analysis:\n{contour_json}"}
                    ]
                }
            ],
            max_tokens=150,
            temperature=0.5,
            top_p=0.8
        )
        result_text = response.choices[0].message.content.strip()
        self.get_logger().info(f"[PATH PLAN RESPONSE]: {result_text}")
        try:
            if result_text.startswith("```"):
                result_text = re.sub(r"```(json)?", "", result_text).strip()
                result_text = re.sub(r"```", "", result_text).strip()
            result = json.loads(result_text)
            return result.get("path", [])
        except Exception as e:
            self.get_logger().error(f"[PATH PLAN ERROR] {e}")
            return []


    def main_process(self):
        try:
            image_path = self.get_latest_image_path()
            if not image_path or not os.path.exists(image_path):
                self.processing = False
                return

            image_data = self.image_to_base64(image_path)
            image = cv2.imread(image_path)
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            image_height, image_width = image.shape[:2]

            hsv_ranges = [
                ((0, 50, 50), (10, 255, 255)),
                ((160, 50, 50), (180, 255, 255)),
                ((20, 50, 50), (40, 255, 255)),
                ((0, 0, 0), (180, 255, 80)),
                ((130, 30, 30), (160, 255, 255)),
                ((35, 50, 50), (85, 255, 255)),
            ]
            masks = [cv2.inRange(hsv, lower, upper) for (lower, upper) in hsv_ranges]
            general_color_mask = masks[0]
            for m in masks[1:]:
                general_color_mask = cv2.bitwise_or(general_color_mask, m)

            top_ignore_y = int(image_height * 0.2)
            bottom_ignore_y = int(image_height * 0.9)
            center_width = int(image_width * 0.3)
            side_width = (image_width - center_width) // 2

            cv2.rectangle(general_color_mask, (0, 0), (image_width, top_ignore_y), 0, -1)
            cv2.rectangle(general_color_mask, (0, bottom_ignore_y), (side_width, image_height), 0, -1)
            cv2.rectangle(general_color_mask, (image_width - side_width, bottom_ignore_y), (image_width, image_height), 0, -1)

            contours, _ = cv2.findContours(general_color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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

            contour_json = json.dumps({"contour_analysis": contour_summary})

            if self.in_path_mode:
                if self.current_step >= len(self.path_plan):
                    self.get_logger().info("[PATH MODE] Path complete. Returning to normal mode.")
                    self.in_path_mode = False
                    self.path_plan = []
                    self.current_step = 0
                    return

                # 2. get_latest_image_path
                image_path = self.get_latest_image_path()
                if not image_path or not os.path.exists(image_path):
                    self.get_logger().warn("[WARNING] No new image found during path execution.")
                    self.in_path_mode = False
                    self.path_plan = []
                    self.current_step = 0
                    return

                image = cv2.imread(image_path)

                # 3. check_threat_in_image
                decision = self.request_threat_assessment_from_image(image_data, image_path, contour_json)

                if decision == "stop":
                    self.get_logger().warn("[THREAT] GPT advised stop during path plan.")
                    self.in_path_mode = False
                    self.path_plan = []
                    self.current_step = 0
                    return

                # 4. move
                direction_raw = self.path_plan[self.current_step]
                direction = str(direction_raw).strip().replace("'", "").replace('"', "")

                if direction in ['w', 'a', 's', 'd']:
                    self.get_logger().info(f"[PATH MODE] Executing step {self.current_step + 1}: {direction}")
                    self.direction_pub.publish(String(data=direction))
                    self.current_step += 1
                elif direction == "stop":
                    self.get_logger().info("[PATH MODE] Received 'stop' command. Ending path mode.")
                    self.stop_pub.publish(String(data="stop"))
                    self.in_path_mode = False
                    self.path_plan = []
                    self.current_step = 0
                else:
                    self.get_logger().warn(f"[PATH MODE] Invalid direction '{direction_raw}' at step {self.current_step}. Skipping.")
                    self.current_step += 1 


                return

            decision, direction, duck_found = self.request_decision_and_direction_from_image(image_data, image_path, contour_json)

            if duck_found:
                self.get_logger().info("[INFO] Duck detected → switching to path planning.")
                self.in_path_mode = True
                self.path_plan = self.request_path_plan_from_image(image_data, image_path, contour_json)
                self.current_step = 0
                if self.path_plan and self.path_plan[0] != "stop":
                    self.direction_pub.publish(String(data=self.path_plan[0]))
                    self.get_logger().info(f"[PATH MODE] Immediately executing step 1: {self.path_plan[0]}")
                    self.current_step += 1
                return

            if decision == "stop":
                self.stop_pub.publish(String(data="stop"))
            elif decision == "move" and direction in ['w', 'a', 's', 'd']:
                self.direction_pub.publish(String(data=direction))

        except Exception as e:
            self.get_logger().error(f"[ERROR] {e}")
        finally:
            self.processing = False


def main(args=None):
    rclpy.init(args=args)
    node = GPTImageRobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
