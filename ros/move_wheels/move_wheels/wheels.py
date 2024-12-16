import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from move_wheels.motor_controller import MotorController
from move_wheels.turret import Turret

# Motor channel definitions
FRONT_LEFT = 0
FRONT_RIGHT = 1
BACK_LEFT = 2
BACK_RIGHT = 3

THETA_LIMIT = (0, 70)
PHI_LIMIT = (0, 70)


class MotorDriver(Node):
    def __init__(self):
        super().__init__("motor_driver")

        self.last_direction = ""

        # Initialize ServoKit for PCA9685
        self.motor = MotorController()
        self.turret = Turret(8, 9, THETA_LIMIT, PHI_LIMIT)
        self.turret_degree = 0

        self.subscription = self.create_subscription(
            String, "/move_cmd", self.cmd_callback, 1
        )

        self.laser_fire = self.create_publisher(Float32, "/fire_lazer", 1)

        self.get_logger().info("Directional Motor Driver Node Initialized")

    def cmd_callback(self, msg):
        direction = msg.data
        speed = 1  # msg.speed or something
        if direction == self.last_direction:
            self.get_logger().info("same as last direction, ignored")
            return
        else:
            self.motor.stop_all()

        self.get_logger().info(f"Direction Received {direction}")

        self.last_direction = direction
        match direction:
            case "fire":
                self.last_direction = ""
                fire = Float32()
                fire.data = 1.0
                self.laser_fire.publish(fire)
            case "turret_down":
                self.last_direction = ""
                try:
                    self.turret.rotate(1, 0)
                except ValueError:
                    pass
            case "turret_up":
                self.last_direction = ""
                try:
                    self.turret.rotate(-1, 0)
                except ValueError:
                    pass
            case "strafe_left":
                self.motor.move_left(speed)
            case "strafe_right":
                self.motor.move_right(speed)
            case "forward":
                self.motor.move_forward(speed)
            case "backward":
                self.motor.move_backward(speed)
            case "turn_left":
                self.motor.rotate_counterclockwise(speed / 3)
            case "turn_right":
                self.motor.rotate_clockwise(speed / 3)
            case "stop":
                self.motor.stop_all()
            case _:
                self.get_logger().warn(f"Unrecoginized instruction: {direction}")

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
