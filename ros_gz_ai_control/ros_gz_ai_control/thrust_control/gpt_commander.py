import os
import re
import base64
import threading
import json
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
            "The camera is mounted 0.85 meters from the front and 1.1 meters above the water surface.\n"
            "The image has been edited to remove unnecessary elements such as the front engine cover of the drone, to simplify perception and reduce misclassification of obstacles.\n"
            "Note: The area hidden at the bottom of the image represents the front engine of the drone. Avoid letting obstacles touch this region even if it is not visible in the image.\n"
            "A gray triangular area is overlaid in the image to indicate the expected forward movement path of the drone.\n"
            "Obstacles within or near this gray path should be carefully avoided.\n\n"
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
            "    2.1 - If the yellow duck is far, move forward ('w') to approach it.\n"
            "    2.2 - If the yellow duck is close, center the yellow duck in the view and stop.\n"
            "3. If there are obstacles on image and obstacles are far:\n"
            "    3.1 - If the yellow duck is not visible, move forward or rotate freely to search the yellow duck.\n"
            "    3.2 - If the yellow duck is visible, move forward in a direction that keeps distance from the obstacles while approaching the yellow duck.\n"
            "4. If there are obstacles on image and obstacles are close:\n"
            "    4.1 - If the yellow duck is not visible, rotate away from the nearest obstacle to find the yellow duck.\n"
            "    4.2 - If the yellow duck is far, move forward only in a direction that turns away from the obstacle.\n"
            "    4.3 - If the yellow duck is close, first adjust the drone to keep away from the obstacle, then rotate or move to center the yellow duck.\n"
            "5. If the yellow duck is centered and its distance is within 2 meters, stop.\n"
            "6. If you find a yellow duck, respond duck_found as true, otherwise false.\n\n"
            "Note: If \"duck_position\" is \"unknown\", then \"duck_found\" must be false.\n"
            
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
            "The camera is mounted 0.85 meters from the front and 1.1 meters above the water surface.\n"
            "The image has been edited to remove unnecessary elements such as the front engine cover of the drone, to simplify perception and reduce misclassification of obstacles.\n"
            "Note: The area hidden at the bottom of the image represents the front engine of the drone. Avoid letting obstacles touch this region even if it is not visible in the image.\n"
            "A gray triangular area is overlaid in the image to indicate the expected forward movement path of the drone.\n"
            "Obstacles within or near this gray path should be carefully avoided.\n\n"
            "Camera has a horizontal field of view (FOV) of 90º and a vertical FOV of 60º. According to this, it would be left 45º if it was on the left-end and right 45º if it was on the right-end.\n"
            "All directional decisions (left/right) must be made based strictly on the image coordinates:\n"
            "- The left side of the image is 'left'.\n"
            "- The right side of the image is 'right'.\n"
            "Your task is to decide the next movement direction based on image\n"
            "Use the following rules:\n"
            "1. If there are no obstacles and the yellow duck is visible:\n"
            "    - If the yellow duck is on the right, rotate right ('d') until it is near the center, then move forward ('w').\n"
            "    - If the yellow duck is on the left, rotate left ('a') until it is near the center, then move forward ('w').\n"
            "    - Do not rotate in the opposite direction of the duck's position.\n"
            "2. If obstacles exist and are far:\n"
            "    - Prioritize approaching the duck while maintaining a safe path.\n"
            "3. If obstacles are close:\n"
            "    - Avoid obstacles using the opposite direction, then continue toward the duck.\n"
            "4. If the yellow duck is centered and within 2 meters, stop.\n\n"
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
                        {"type": "text", "text": "Make the decision for the drone to reach the yellow duck. Yellow duck is not an obstacle, so don't avoid it."
                                            "Use image contents to determine presence.\n"
                                            "Place the yellow duck at the center-bottom of the image\n"
                                            "The rotation of direction doesn't necessarily have to be one. There's no problem with multiple times.\n"
                                            "And it's also possible to move back through s.\n"}
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

                image_data = self.image_to_base64(image_path)
                
                # 3. check_threat_in_image
                decision = self.request_threat_assessment_from_image(image_data, image_path)

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