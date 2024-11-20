import rclpy
from rclpy.node import Node
from gpiozero import OutputDevice
from std_msgs.msg import Float32
import time


class LaserControlNode(Node):
    def __init__(self):
        super().__init__("laser_control_node")
        self.laser = OutputDevice(24)  # CHANGE NUMBER TO USED GPIO PIN

        self.subscription = self.create_subscription(
            Float32, "/fire_lazer", self.laser_callback, 10
        )
        self.get_logger().info("Laser control node has started")

    def fire(self, duration):
        self.laser.on()
        time.sleep(duration)
        self.laser.off()

    def laser_callback(self, msg):
        duration = msg.data
        self.get_logger().info(f"Firing laser for {duration} seconds.")
        self.fire(duration)


def main(args=None):
    rclpy.init(args=args)
    laser_control_node = LaserControlNode()

    rclpy.spin(laser_control_node)

    laser_control_node.destroy_node()
    rclpy.shutdown()
