import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
import math
import time


class TelemetryPublisher(Node):
    def __init__(self):
        super().__init__('telemetry_publisher')

        self.velocity_pub = self.create_publisher(Vector3, 'velocity', 10)
        self.battery_pub = self.create_publisher(Float32, 'battery', 10)

        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.publish_telemetry)

    def publish_telemetry(self):
        t = time.time() - self.start_time

        velocity = Vector3()
        velocity.x = math.sin(t)
        velocity.y = math.cos(t)
        velocity.z = 0.0

        battery = Float32()
        battery.data = max(0.0, 100.0 - t)

        self.velocity_pub.publish(velocity)
        self.battery_pub.publish(battery)

        self.get_logger().info(
            f'Publishing velocity=({velocity.x:.2f}, {velocity.y:.2f}) battery={battery.data:.1f}%'
        )


def main():
    rclpy.init()
    node = TelemetryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
