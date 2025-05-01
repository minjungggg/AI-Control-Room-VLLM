import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import numpy as np
import json

class IMUVelocityEstimator(Node):
    def __init__(self):
        super().__init__('imu_velocity_estimator')

        self.subscription = self.create_subscription(
            Imu,
            '/world/waves/model/wamv_camera/link/imu_link/sensor/imu_sensor/imu',
            self.imu_callback,
            10
        )

        self.velocity_pub = self.create_publisher(String, '/velocity_json', 10)

        self.linear_velocity = np.array([0.0, 0.0, 0.0])   # [vx, vy, vz]
        self.angular_velocity = np.array([0.0, 0.0, 0.0])  # [roll_rate, pitch_rate, yaw_rate]
        self.prev_time = None

        self.get_logger().info("IMU Velocity Estimator Node Started")

    def imu_callback(self, msg: Imu):
        current_time = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_time is None:
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time
        self.prev_time = current_time

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z + 9.81
        accel = np.array([ax, ay, az])
        self.linear_velocity += accel * dt

        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z
        self.angular_velocity = np.array([wx, wy, wz])

        # JSON message format
        velocity_data = {
            "linear_velocity": {
                "x": float(self.linear_velocity[0]),
                "y": float(self.linear_velocity[1]),
                "z": float(self.linear_velocity[2])
            },
            "angular_velocity": {
                "x": float(self.angular_velocity[0]),
                "y": float(self.angular_velocity[1]),
                "z": float(self.angular_velocity[2])
            }
        }

        msg_json = String()
        msg_json.data = json.dumps(velocity_data)
        self.velocity_pub.publish(msg_json)

        # self.get_logger().info(f"Published velocity JSON: {msg_json.data}")

def main(args=None):
    rclpy.init(args=args)
    node = IMUVelocityEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
