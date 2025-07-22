import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
import numpy as np
import cv2
import os

class FastImageSaver(Node):
    def __init__(self):
        super().__init__('fast_image_saver')
        self.bridge = CvBridge()
        self.image_index = 0
        self.counter = 0
        self.save_interval = 64
        self.received_camera_info = False
        self.map1 = None
        self.map2 = None

        # Declare and get parameters
        self.declare_parameter('save_interval', 64)
        self.save_interval = self.get_parameter('save_interval').get_parameter_value().integer_value

        # Save path
        self.image_save_path = os.path.expanduser('~/saved_images')
        os.makedirs(self.image_save_path, exist_ok=True)
        self.get_logger().info("FastImageSaver Node (Remap-Based Undistortion) Started")

        # Subscriptions
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/world/waves/model/wamv_camera/link/camera_link/sensor/camera_sensor/camera_info',
            self.camera_info_callback,
            10)

        self.image_sub = self.create_subscription(
            Image,
            '/world/waves/model/wamv_camera/link/camera_link/sensor/camera_sensor/image',
            self.image_callback,
            10)

        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'save_interval':
                self.save_interval = param.value
                self.get_logger().info(f"Updated save_interval to {self.save_interval}")
        return SetParametersResult(successful=True)

    def camera_info_callback(self, msg: CameraInfo):
        if not self.received_camera_info:
            k = np.array(msg.k).reshape((3, 3))
            d = np.array(msg.d)
            image_size = (msg.width, msg.height)

            # 보정 맵 생성 (최초 1회)
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                cameraMatrix=k,
                distCoeffs=d,
                R=np.eye(3),
                newCameraMatrix=k,
                size=image_size,
                m1type=cv2.CV_16SC2
            )
            self.received_camera_info = True
            # self.get_logger().info("CameraInfo received and undistort maps initialized.")

    def image_callback(self, msg: Image):
        self.counter += 1
        if self.counter % self.save_interval != 0:
            return

        if not self.received_camera_info:
            self.get_logger().warn("Skipping image: CameraInfo not yet received.")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgra8')

            undistorted_image = cv2.remap(cv_image, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)

            # # 엔진 부분 마스킹해서 가리기
            # image_height, image_width = undistorted_image.shape[:2]
            # unit = image_width / 63
            
            # top_ignore_y = int(image_height * 0.2)
            # bottom_ignore_y = int(image_height * 0.9)
            # x1 = int(unit * 7)
            # x2 = int(unit * (7 + 8))
            # x3 = int(unit * (7 + 8 + 33))
            # x4 = int(unit * (7 + 8 + 33 + 8))
            
            # mask = np.ones((image_height, image_width), dtype=np.uint8) * 255
            # cv2.rectangle(mask, (0, 0), (image_width, top_ignore_y), 0, -1)
            # cv2.rectangle(mask, (x1, bottom_ignore_y), (x2, image_height), 0, -1)  # left engine
            # cv2.rectangle(mask, (x3, bottom_ignore_y), (x4, image_height), 0, -1)  # right engine

            # masked_image = cv2.bitwise_and(undistorted_image, undistorted_image, mask=mask)

            
            # # 드론의 이동 경로 제작
            # image_height, image_width = undistorted_image.shape[:2]
            # unit = image_width / 63
            
            # x1 = int(unit * 7)                             # left engine START
            # x4 = int(unit * (7 + 8 + 33 + 8))              # right engine END
            # vanishing_point = (image_width // 2, image_height // 2)  # 화면 중심
            
            # triangle = np.array([[
            #     (x1, image_height),       
            #     (x4, image_height),      
            #     vanishing_point       
            # ]], dtype=np.int32)
            
            # overlay = masked_image.copy()
            # cv2.fillPoly(overlay, triangle, color=(127, 127, 127))  # BGR 회색
            
            # alpha = 0.3         # 숫자가 작아지면 투명도가 높아짐
            # blended = cv2.addWeighted(overlay, alpha, masked_image, 1 - alpha, 0)
            
            filename = os.path.join(self.image_save_path, f"saved_undistorted_image_{self.image_index}.png")
            cv2.imwrite(filename, undistorted_image)
            
            self.get_logger().info(f"Saved undistorted image with remap: {filename}")
            self.image_index += 1

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FastImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()