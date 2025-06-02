import os
import re
import base64
import threading
import json
import cv2
from collections import deque
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
        self.prev_observations = deque(maxlen=3)
        self.last_known_duck_position = "unknown"

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

    def record_observation(self, decision, direction, duck_found, duck_position):
        obs = {
            "decision": decision,
            "direction": direction,
            "duck_found": duck_found,
            "duck_position": duck_position
        }
        self.prev_observations.appendleft(obs)

    def get_prev_summary_note(self):
        if not self.prev_observations:
            return "No previous observation available."

        note = "Recent observations:\n"
        for idx, obs in enumerate(self.prev_observations, 1):
            note += f"{idx} info:\n"
            note += json.dumps({
                "decision": obs["decision"],
                "direction": obs["direction"],
                "duck_found": obs["duck_found"],
                "duck_position": obs["duck_position"]
            }, ensure_ascii=False)
            note += "\n"
        return note

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

    def request_threat_assessment_from_image(self, image_data, image_path):
        prompt = (
            "You are the navigation system of an autonomous water drone.\n"
            "The drone is twin-hull (catamaran-style), 2.5m wide, 5m long, and 1.5m high.\n"
            "The camera is mounted 0.85 meters from the front and 1.1 meters above the water surface.\n\n"
            "Your task is to decide whether the drone should STOP or continue MOVE, based on obstacles and the yellow duck position (if visible).\n\n"
            "Follow these rules:\n"
            "- If any object (e.g., buoy, obstacle **except duck**) is directly in front of the drone and appears within approximately 2 meters, respond with \"stop\".\n"
            "- If the yellow duck is centered and very close (within ~2 meters), also respond with \"stop\".\n"
            "- If the path ahead looks clear, even if the duck is not visible, respond with \"move\".\n"
            "- If you are unsure, prefer \"move\" over \"stop\".\n\n"
            "Do not be overly cautious. Base your judgment on clear visual threat of collision.\n\n"
            "Respond ONLY with the following JSON format (no explanations or markdown):\n"
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
                            "Evaluate based only on this image.\n"
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


    def request_decision_and_direction_from_image(self, image_data, image_path):
        self.get_logger().info(f"*****************************{image_path}*****************************")
        prompt = (
            "You are the navigation system of an autonomous water drone.\n"
            "The drone is twin-hull (catamaran-style), 2.5m wide, 5m long, and 1.5m high.\n"
            "The camera is mounted 0.85 meters from the front and 1.1 meters above the water surface.\n\n"
            "Camera has a horizontal field of view (FOV) of 90º and a vertical FOV of 60º."
            "According to this, it would be left 45º if it was on the left-end and right 45º if it was on the right-end.\n"
            "The duck_position should describe where the yellow duck appears in the image using approximate angular position from the center.\n"
            "Use one of the following formats:"
            "   - \"left-30º\", \"left-15º\", \"center\", \"right-10º\", \"right-25º\""
            "   - If no duck is visible, respond with: \"unknown\""
            
            "Your task is to decide the next movement direction based on the current image and recent navigation history.\n\n"
            
            "Primary rules based on the current image:\n"
            "1. If there are not obstacles and yellow duck on image, rotate ('a' or 'd') to search the yellow duck.\n"
            "2. If there are not obstacles but the yellow duck is visible.:\n"
            "    2.1 - If the yellow duck is far (>10m), move forward ('w') to approach it.\n"
            "    2.2 - If the yellow duck is close (≤10m), center the yellow duck in the view and stop.\n"
            "3. If there are obstacles on image and obstacles are far (>8m):\n"
            "    3.1 - If the yellow duck is not visible, move forward or rotate freely to search the yellow duck.\n"
            "    3.2 - If the yellow duck is visible, move forward in a direction that keeps distance from the obstacles while approaching the yellow duck.\n"
            "4. If there are obstacles on image and obstacles are close (≤8m):\n"
            "    4.1 - If the yellow duck is not visible, rotate away from the nearest obstacle to find the yellow duck.\n"
            "    4.2 - If the yellow duck is far (>10m), move forward only in a direction that turns away from the obstacle.\n"
            "    4.3 - If the yellow duck is close (≤10m), first adjust the drone to keep away from the obstacle, then rotate or move to center the yellow duck.\n"
            "5. If the yellow duck is centered and its distance is within 2 meters, stop.\n"
            "6. If you find a yellow duck, respond duck_found as true, otherwise false.\n\n"
            "Note: If \"duck_position\" is \"unknown\", then \"duck_found\" must be false.\n"
            
            "Supplementary logic using recent decision history (only apply if the image does not clearly show the duck):\n"
            "- If the duck was recently visible (e.g., duck_found : true & duck_position : left-15º) but now missing, consider reversing direction or retracing steps.\n"
            "- If the duck has not been seen for multiple steps, try continuously rotating in the same direction (e.g., keep turning left 'a' for several steps).\n"
            "- Avoid repeating the same short back-and-forth pattern (e.g., a → d → a → d).\n"
            "- Avoid repeating the same direction repeatedly when the duck is not found.\n"
            "- Always prioritize decisions based on clear, visible objects in the current image. Use history only if uncertain.\n\n"
            
            "Respond strictly in the following JSON format:\n"
            "Do not include any explanations, markdown formatting, or code block markers like ```json. "
            "Output only the raw JSON object."
            "{\n"
            "  \"decision\": \"move\" or \"stop\",\n"
            "  \"direction\": \"w\" or \"a\" or \"s\" or \"d\"\n"
            "  \"duck_found\": true or false\n"
            "  \"duck_position\": a string such as \"unknown\" or \"left-20º\" or \"right-10º\" or \"center\" \n"
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
                        {"type": "text", "text": ("I want to get to the yellow duck if it exists, while avoiding obstacles.\n"
                                            "Only identify a yellow duck if it is clearly present in the image.\n"
                                            "Do not assume a yellow duck is always there. "
                                            "Use image contents to determine presence.\n"
                                            "Place the yellow duck at the center-bottom of the image **only if found**.\n"
                                            "-------------------------------------------------\n"
                                            "Below is a summary of the previous 3 observations made by the navigation system, listed in chronological order:\n"
                                            "    - 1 info: most recent observation (just before the current image)\n"
                                            "    - 2 info: one step before that\n"
                                            "    - 3 info: the oldest of the last three observations\n\n"
                                            "Each observation includes:\n"
                                            "    - decision: whether the drone moved or stopped\n"
                                            "    - direction: which direction the drone moved (w, a, s, or d)\n"
                                            "    - duck_found: whether a yellow duck was found in that step\n"
                                            "    - duck_position: estimated angular position of the duck (if found)\n"
                                            "    - path(option): a planned movement sequence, only present if the duck was found and path planning was triggered\n\n"
                                            "Use this historical information when instructed by the rules above (e.g., Rule 1, 3.1, and 4.1).\n"
                                            "-------------------------------------------------\n"
                                            + self.get_prev_summary_note() + "\n"
                                            )}
                    ]
                }
            ],
            max_tokens=100,
            temperature=0.7,
            top_p=0.5
        )
        result_text = response.choices[0].message.content.strip()
        self.get_logger().info(f"GPT response: {result_text}")
        try:
            if result_text.startswith("```"):
                result_text = re.sub(r"```(json)?", "", result_text).strip()
                result_text = re.sub(r"```", "", result_text).strip()
            result = json.loads(result_text)
            return result.get("decision", ""), result.get("direction", ""), result.get("duck_found", False), result.get("duck_position", "unknown")
        except:
            return None, None, False, "unknown"

    def request_path_plan_from_image(self, image_data, image_path):
        self.get_logger().info(f"[PATH PLAN] From {image_path}")
        prompt = (
            "You are the navigation system of an autonomous water drone.\n"
            "The drone is twin-hull (catamaran-style), 2.5m wide, 5m long, and 1.5m high.\n"
            "The camera is mounted 0.85 meters from the front and 1.1 meters above the water surface.\n\n"
            "Camera has a horizontal field of view (FOV) of 90º and a vertical FOV of 60º. According to this, it would be left 45º if it was on the left-end and right 45º if it was on the right-end.\n"
            "Your task is to decide the next movement direction based on image\n"
            "Use the following rules:\n"
            "1. If there are not obstacles and yellow duck on image, rotate ('a' or 'd') to search the yellow duck.\n"
            "2. If there are not obstacles but the yellow duck is visible.:\n"
            "    2.1 - If the yellow duck is far (>10m), move forward ('w') to approach it.\n"
            "    2.2 - If the yellow duck is close (≤10m), center the yellow duck in the view and stop.\n"
            "3. If there are obstacles on image and obstacles are far (>8m):\n"
            "    3.1 - If the yellow duck is not visible, move forward or rotate freely to search the yellow duck.\n"
            "    3.2 - If the yellow duck is visible, move forward in a direction that keeps distance from the obstacles while approaching the yellow duck.\n"
            "4. If there are obstacles on image and obstacles are close (≤8m):\n"
            "    4.1 - If the yellow duck is not visible, rotate away from the nearest obstacle to find the yellow duck.\n"
            "    4.2 - If the yellow duck is far (>10m), move forward only in a direction that turns away from the obstacle.\n"
            "    4.3 - If the yellow duck is close (≤10m), first adjust the drone to keep away from the obstacle, then rotate or move to center the yellow duck.\n"
            "5. If the yellow duck is centered and its distance is within 2 meters, stop.\n"
            "6. If the yellow duck is on the left or right side of the image, and rotating ('a' or 'd') toward it does not lead to collision with nearby obstacles, include the rotation in the path plan. If rotating toward the duck would lead to a collision, avoid that direction.\n\n"
            "Respond strictly in the following JSON format:\n"
            "Do not include any explanations, markdown formatting, or code block markers like ```json. "
            "Output only the raw JSON object."
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
                        {"type": "text", "text": "I want to get to the yellow duck while avoiding obstacles.\n"
                                            "Use image contents to determine presence.\n"
                                            "Place the yellow duck at the center-bottom of the image\n"}
                    ]
                }
            ],
            max_tokens=40,
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
            unit = image_width / 63
            x1 = int(unit * 7)        # left engine
            x2 = int(unit * (7 + 8))  # left engine
            x3 = int(unit * (7 + 8 + 33))      # right engine
            x4 = int(unit * (7 + 8 + 33 + 8))  # right engine

            cv2.rectangle(general_color_mask, (0, 0), (image_width, top_ignore_y), 0, -1)
            cv2.rectangle(general_color_mask, (x1, bottom_ignore_y), (x2, image_height), 0, -1)  # left engine
            cv2.rectangle(general_color_mask, (x3, bottom_ignore_y), (x4, image_height), 0, -1)  # right engine

            if self.in_path_mode:
                if self.current_step >= len(self.path_plan):
                    self.get_logger().info("[PATH MODE] Path complete. Returning to normal mode.")
                    self.record_observation(
                        decision="move",
                        direction=self.path_plan,  # Full plan completed
                        duck_found=True,
                        duck_position=self.last_known_duck_position
                    )
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

                image_data = self.image_to_base64(image_path)
                
                # 3. check_threat_in_image
                decision = self.request_threat_assessment_from_image(image_data, image_path)

                if decision == "stop":
                    executed_path = self.path_plan[:self.current_step]
                    self.get_logger().warn("[THREAT] GPT advised stop during path plan.")
                    self.record_observation(
                        decision="stop",
                        direction=executed_path,
                        duck_found=True,
                        duck_position=self.last_known_duck_position
                    )
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
                else:
                    self.get_logger().warn(f"[PATH MODE] Invalid direction '{direction_raw}' at step {self.current_step}. Skipping.")
                    self.current_step += 1 

                return

            decision, direction, duck_found, duck_position = self.request_decision_and_direction_from_image(image_data, image_path, )
            self.last_known_duck_position = duck_position
            
            if duck_found:
                self.get_logger().info("[INFO] Duck detected → switching to path planning.")
                self.in_path_mode = True
                
                image_path = self.get_latest_image_path()
                image_data = self.image_to_base64(image_path)
                
                self.path_plan = self.request_path_plan_from_image(image_data, image_path)
                self.current_step = 0
                
                # Record the observation before executing the path plan
                self.record_observation(decision, [direction], duck_found, duck_position)
                
                if self.path_plan and self.path_plan[0] != "stop":
                    self.direction_pub.publish(String(data=self.path_plan[0]))
                    self.get_logger().info(f"[PATH MODE] Immediately executing step 1: {self.path_plan[0]}")
                    self.current_step += 1
                return
            
            self.record_observation(decision, [direction], duck_found, duck_position)

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