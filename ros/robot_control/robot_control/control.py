import rclpy
from rclpy.node import Node
from robot_interfaces.msg import TurretInstruction, WheelsInstruction
from .motor_controller import MotorController
from .turret import Turret
from gpiozero import OutputDevice
import time

# Motor channel definitions
THETA_LIMIT = (0, 70)
PHI_LIMIT = (0, 180)
LASER_DELAY = 3  # seconds


class MotorDriver(Node):
    def __init__(self):
        super().__init__("motor_driver")

        self.last_direction = ""

        # Initialize ServoKit for PCA9685
        self.motor = MotorController()
        self.turret = Turret(8, 9, THETA_LIMIT, PHI_LIMIT, 45)
        self.laser = OutputDevice(17)  # CHANGE NUMBER TO USED GPIO PIN
        self.last_laser_fire = int(time.time())

        self.wheels_sub = self.create_subscription(
            WheelsInstruction, "/wheels_cmd", self.wheels_callback, 1
        )

        self.turret_sub = self.create_subscription(
            TurretInstruction, "/turret_cmd", self.turret_callback, 1
        )

        self.get_logger().info("Directional Motor Driver Node Initialized")

    def wheels_callback(self, msg: WheelsInstruction):
        direction = msg.direction
        speed = msg.speed
        if direction == self.last_direction:
            self.get_logger().info("same as last direction, ignored")
            return
        else:
            self.motor.stop_all()

        self.get_logger().info(f"Direction Received {direction}")

        self.last_direction = direction
        match direction:
            case WheelsInstruction.STRAFE_LEFT:
                self.motor.move_left(speed)
            case WheelsInstruction.STRAFE_RIGHT:
                self.motor.move_right(speed)
            case WheelsInstruction.FORWARD:
                self.motor.move_forward(speed)
            case WheelsInstruction.BACKWARD:
                self.motor.move_backward(speed)
            case WheelsInstruction.TURN_LEFT:
                self.motor.rotate_counterclockwise(speed / 3)
            case WheelsInstruction.TURN_RIGHT:
                self.motor.rotate_clockwise(speed / 3)
            case WheelsInstruction.STOP:
                self.motor.stop_all()
            case _:
                self.get_logger().warn(f"Unrecoginized wheels instruction: {direction}")
                self.motor.stop_all()

    def turret_callback(self, msg: TurretInstruction):
        if msg.laser_duration > 0 and (int(time.time()) - self.last_laser_fire > LASER_DELAY
        ):
            self.last_laser_fire = int(time.time())
            self.get_logger().info(f"firing my laser for {msg.laser_duration}")
            self.laser.on()
            time.sleep(msg.laser_duration)
            self.laser.off()

        if msg.zero_turret:
            try:
                self.turret.slow_zero()
            except ValueError:
                pass
        else:
            try:
                self.turret.rotate(msg.theta, msg.phi)
            except ValueError:
                pass

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
