# image_saver.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import os
from cv_bridge import CvBridge

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/world/waves/model/wamv_camera/link/camera_link/sensor/camera_sensor/image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.timer_period = 10.0  # 이후 저장 간격 (초)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.image_msg = None
        self.image_count = 1  # 1부터 시작 (1번 이미지는 즉시 저장)
        self.first_image_saved = False

        home = os.path.expanduser("~")
        self.save_dir = os.path.join(home, 'Desktop', 'image_saver')
        os.makedirs(self.save_dir, exist_ok=True)

    def listener_callback(self, msg):
        self.image_msg = msg

        # 첫 이미지가 수신되었고, 아직 저장되지 않았다면 바로 저장
        if not self.first_image_saved:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                filename = f"saved_image_1.png"
                filepath = os.path.join(self.save_dir, filename)
                cv2.imwrite(filepath, cv_image)
                self.get_logger().info(f'[즉시저장] 이미지 저장됨: {filename}')
                self.first_image_saved = True
                self.image_count = 2  # 다음 저장부터는 2번부터 시작
            except Exception as e:
                self.get_logger().error(f'[즉시저장] 이미지 저장 중 오류 발생: {str(e)}')

    def timer_callback(self):
        if self.image_msg is not None and self.first_image_saved:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding='bgr8')
                filename = f"saved_image_{self.image_count}.png"
                filepath = os.path.join(self.save_dir, filename)
                cv2.imwrite(filepath, cv_image)
                self.get_logger().info(f'[주기저장] 이미지 저장됨: {filename}')
                self.image_count += 1
            except Exception as e:
                self.get_logger().error(f'[주기저장] 이미지 저장 중 오류 발생: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
