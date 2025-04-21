import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')
        # 해당 이미지 토픽을 구독합니다.
        self.subscription = self.create_subscription(
            Image,
            '/world/waves/model/wamv_camera/link/camera_link/sensor/camera_sensor/image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info("Image Viewer Node Started")

    def listener_callback(self, msg):
        try:
            # sensor_msgs/Image 메시지를 OpenCV 이미지로 변환 (rgb8 인코딩)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')
            # OpenCV 창으로 이미지 출력
            cv2.imshow("Camera Image", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error("이미지 변환 오류: %s" % str(e))

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
