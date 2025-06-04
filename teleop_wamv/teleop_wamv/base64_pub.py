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

    def wait_for_file_complete(self, file_path, timeout=2.0):
        prev_size = -1
        start_time = time.time()
        while time.time() - start_time < timeout:
            current_size = os.path.getsize(file_path)
            if current_size == prev_size:
                return True  # 파일 크기 고정됨 = 저장 완료
            prev_size = current_size
            time.sleep(0.1)
        return False

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
        
        if not self.wait_for_file_complete(latest_image_path):
            self.get_logger().warn(f'파일 저장이 완료되지 않음: {latest_image_path}')
            return

        # 파일 크기 제한 (5MB 이하 권장)
        if os.path.getsize(latest_image_path) > 5 * 1024 * 1024:
            self.get_logger().warn(f'파일 크기 초과 (5MB 이상): {latest_image_path}')
            return
        
        try:
            with open(latest_image_path, "rb") as image_file:
                encoded_string = base64.b64encode(image_file.read()).decode('utf-8')
        except Exception as e:
            self.get_logger().warn(f'base64 인코딩 실패: {e}')
            return
        
        msg = String()
        msg.data = encoded_string
        self.publisher_.publish(msg)
        self.last_published = latest_image_path
 
            
def main(args=None):
    rclpy.init(args=args)
    node = Base64Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()