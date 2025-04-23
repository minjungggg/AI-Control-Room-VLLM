import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import base64
import os
import time

class Base64Publisher(Node):
    def __init__(self):
        super().__init__('base64_pub')
        self.publisher_ = self.create_publisher(String, 'image_base64', 10)
        timer_period = 5.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.last_published = ''
        self.image_dir = os.path.expanduser('~/Desktop/image_saver')

    def timer_callback(self):
        if not os.path.exists(self.image_dir):
            self.get_logger().warn(f'이미지 폴더 없음: {self.image_dir}')
            return

        image_files = sorted(
            [f for f in os.listdir(self.image_dir) if f.endswith('.png')],
            key=lambda x: os.path.getmtime(os.path.join(self.image_dir, x))
        )

        if not image_files:
            self.get_logger().warn('이미지 파일 없음')
            return

        latest_image_path = os.path.join(self.image_dir, image_files[-1])

        if latest_image_path == self.last_published:
            return

        with open(latest_image_path, "rb") as image_file:
            encoded_string = base64.b64encode(image_file.read()).decode('utf-8')
            msg = String()
            msg.data = encoded_string
            self.publisher_.publish(msg)
            self.last_published = latest_image_path

