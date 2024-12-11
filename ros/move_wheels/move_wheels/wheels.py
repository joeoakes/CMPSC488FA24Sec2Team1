import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from move_wheels.motor_controller import MotorController

# Motor channel definitions
FRONT_LEFT = 0
FRONT_RIGHT = 1
BACK_LEFT = 2
BACK_RIGHT = 3


class MotorDriver(Node):
    def __init__(self):
        super().__init__("motor_driver")

        self.last_direction = ""

        # Initialize ServoKit for PCA9685
        self.motor = MotorController()

        self.subscription = self.create_subscription(
            String, "/move_cmd", self.cmd_callback, 10
        )

        self.get_logger().info("Directional Motor Driver Node Initialized")

    def cmd_callback(self, msg):
        direction = msg.data
        speed = 1  # msg.speed or something
        if direction == self.last_direction:
            return
        else:
            self.motor.stop_all()

        self.get_logger().info(f"Direction Received {direction}")

        match direction:
            case "strafe_left":
                self.motor.move_left(speed)
            case "strafe_right":
                self.motor.move_right(speed)
            case "forward":
                self.motor.move_forward(speed)
            case "backward":
                self.motor.move_backward(speed)
            case "turn_left":
                self.motor.rotate_clockwise(speed)
            case "turn_right":
                self.motor.rotate_counterclockwise(speed)
            case "stop":
                self.motor.stop_all()
            case _:
                self.get_logger().warn(f"Unrecoginized instruction: {direction}")

        self.last_direction = direction

    def destroy(self):
        self.motor.stop_all()


def main(args=None):
    rclpy.init(args=args)
    driver = MotorDriver()

    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        driver.get_logger().info("Shutting down")
    finally:
        driver.destroy_node()
        driver.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
