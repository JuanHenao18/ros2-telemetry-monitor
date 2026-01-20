import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
import math


class TelemetryMonitor(Node):
    def __init__(self):
        super().__init__('telemetry_monitor')

        self.create_subscription(Vector3, 'velocity', self.velocity_callback, 10)
        self.create_subscription(Float32, 'battery', self.battery_callback, 10)

        self.latest_velocity = None
        self.latest_battery = None

    def velocity_callback(self, msg):
        speed = math.sqrt(msg.x**2 + msg.y**2 + msg.z**2)
        self.latest_velocity = speed
        self.report()

    def battery_callback(self, msg):
        self.latest_battery = msg.data
        self.report()

    def report(self):
        if self.latest_velocity is None or self.latest_battery is None:
            return

        warning = "LOW BATTERY!" if self.latest_battery < 20.0 else ""

        self.get_logger().info(
            f'Speed={self.latest_velocity:.2f} m/s | Battery={self.latest_battery:.1f}% {warning}'
        )


def main():
    rclpy.init()
    node = TelemetryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
