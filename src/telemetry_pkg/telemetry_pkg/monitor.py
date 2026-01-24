import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
import math

from telemetry_pkg.utils import TelemetryLogger


class TelemetryMonitor(Node):
    def __init__(self):
        super().__init__('telemetry_monitor')

        self.logger = TelemetryLogger()

        self.create_subscription(Vector3, 'velocity', self.velocity_callback, 10)
        self.create_subscription(Float32, 'battery', self.battery_callback, 10)

        self.speed = None
        self.battery = None

    def velocity_callback(self, msg):
        self.speed = math.sqrt(msg.x**2 + msg.y**2 + msg.z**2)
        self._report()

    def battery_callback(self, msg):
        self.battery = msg.data
        self._report()

    def _report(self):
        if self.speed is None or self.battery is None:
            return

        warning = "LOW BATTERY!" if self.battery < 20.0 else ""

        self.get_logger().info(
            f'Speed={self.speed:.2f} m/s | Battery={self.battery:.1f}% {warning}'
        )

        self.logger.log(self.speed, self.battery)

    def destroy_node(self):
        self.logger.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = TelemetryMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()