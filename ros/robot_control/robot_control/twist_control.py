import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from move_wheels.motor_controller import MotorController


class TwistToMotorDriver(Node):
    def __init__(self):
        super().__init__("twist_to_motor_driver")

        self.motor = MotorController()

        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        self.get_logger().info("Twist to Motor Driver Node Initialized")

    def cmd_vel_callback(self, msg):
        pass
        # if msg.angular.z > 0:
        #     self.motor.rotate_clockwise(1)
        # elif msg.angular.z < 0:
        #     self.motor.rotate_counterclockwise(1)

    def destroy(self):
        self.motor.stop_all()


def main(args=None):
    rclpy.init(args=args)
    driver = TwistToMotorDriver()
    rclpy.spin(driver)
