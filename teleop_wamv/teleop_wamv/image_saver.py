import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import os
from cv_bridge import CvBridge

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.image_sub = self.create_subscription(
            Image,
            '/world/waves/model/wamv_camera/link/camera_link/sensor/camera_sensor/image',
            self.image_callback,
            10)
        self.trigger_sub = self.create_subscription(
            String,
            'save_image_trigger',
            self.trigger_callback,
            10
        )
        self.bridge = CvBridge()
        self.image_msg = None
        self.image_count = 1
        self.first_image_saved = False

        home = os.path.expanduser("~")
        self.save_dir = os.path.join(home, 'Desktop', 'image_saver')
        os.makedirs(self.save_dir, exist_ok=True)

    def image_callback(self, msg):
        self.image_msg = msg
        if not self.first_image_saved:
            self.save_image()
            self.first_image_saved = True
            self.image_count = 2

    def trigger_callback(self, msg):
        if msg.data.strip().lower() == 'next' and self.first_image_saved:
            self.save_image()

    def save_image(self):
        if self.image_msg is None:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding='bgr8')
            filename = f"saved_image_{self.image_count}.png"
            filepath = os.path.join(self.save_dir, filename)
            cv2.imwrite(filepath, cv_image)
            tag = '[즉시저장]' if self.image_count == 1 else '[트리거저장]'
            self.get_logger().info(f'{tag} 이미지 저장됨: {filename}')
            self.image_count += 1
        except Exception as e:
            self.get_logger().error(f'이미지 저장 중 오류 발생: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
