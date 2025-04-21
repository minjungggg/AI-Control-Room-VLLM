import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from rcl_interfaces.msg import SetParametersResult

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        # 이미지 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            '/world/waves/model/wamv_camera/link/camera_link/sensor/camera_sensor/image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.counter = 0
        self.image_index = 0  # 저장될 이미지 번호

        # 파라미터 선언 (기본값: 48) - 약 8초
        self.declare_parameter('save_interval', 96)
        self.save_interval = self.get_parameter('save_interval').get_parameter_value().integer_value

        # 이미지 저장 경로 설정
        self.image_save_path = os.path.expanduser('~/saved_images')
        os.makedirs(self.image_save_path, exist_ok=True)
        self.get_logger().info("Image Saver Node Started")

        # 파라미터 변경 콜백 등록
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'save_interval':
                self.save_interval = param.value
                self.get_logger().info(f"Updated save_interval to {self.save_interval}")
        return SetParametersResult(successful=True)

    def listener_callback(self, msg):
        self.counter += 1
        if self.counter % self.save_interval != 0:
            return

        try:
            # OpenCV 이미지로 변환 (bgra8 인코딩)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')
            # 파일 이름 생성: bgra8_saved_image_0.png, 1.png, ...
            filename = os.path.join(self.image_save_path, f"bgra8_saved_image_{self.image_index}.png")
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"Saved image: {filename}")
            self.image_index += 1
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
