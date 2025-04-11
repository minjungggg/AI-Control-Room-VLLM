import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        # 이미지 토픽을 구독합니다.
        self.subscription = self.create_subscription(
            Image,
            '/world/waves/model/wamv_camera/link/camera_link/sensor/camera_sensor/image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.counter = 0
        self.save_interval = 48  # 매 40번째 프레임 저장 (필요에 따라 변경)
        self.image_save_path = os.path.expanduser('~/saved_images')
        os.makedirs(self.image_save_path, exist_ok=True)
        # 고정된 파일 이름 지정 (덮어쓰기)
        self.filename = os.path.join(self.image_save_path, "latest_image.png")
        self.get_logger().info("Image Saver Node Started")

    def listener_callback(self, msg):
        self.counter += 1
        # 지정된 주기에 맞을 때마다 이미지를 저장
        if self.counter % self.save_interval != 0:
            return

        try:
            # sensor_msgs/Image 메시지를 OpenCV 이미지로 변환 (bgr8 인코딩)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # 고정된 파일 이름으로 이미지 저장 (이전 파일은 자동 덮어쓰여짐)
            cv2.imwrite(self.filename, cv_image)
            self.get_logger().info(f"Saved image (overwritten): {self.filename}")
        except Exception as e:
            self.get_logger().error("이미지 변환 오류: %s" % str(e))

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
