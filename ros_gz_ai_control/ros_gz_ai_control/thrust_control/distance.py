import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
import math

class CumulativeDistanceNode(Node):
    def __init__(self):
        super().__init__('cumulative_distance_node')

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/model/wamv_camera/pose',
            self.pose_callback,
            10
        )

        self.clock_subscription = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )

        self.prev_position = None
        self.total_distance = 0.0
        self.latest_sim_time_sec = 0

    def pose_callback(self, msg: PoseStamped):
        pos = msg.pose.position
        current_position = (pos.x, pos.y, pos.z)

        if self.prev_position is None:
            self.prev_position = current_position
            return

        dx = current_position[0] - self.prev_position[0]
        dy = current_position[1] - self.prev_position[1]
        dz = current_position[2] - self.prev_position[2]

        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        self.total_distance += distance

        self.prev_position = current_position

    def clock_callback(self, msg: Clock):
        self.latest_sim_time_sec = msg.clock.sec

    def on_shutdown(self):
        print(f"\033[91m[FINAL] Cumulative Distance: {self.total_distance:.3f} meters\033[0m")
        print(f"\033[92m[FINAL] Simulation Time: {self.latest_sim_time_sec} seconds\033[0m")

def main(args=None):
    rclpy.init(args=args)
    node = CumulativeDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
