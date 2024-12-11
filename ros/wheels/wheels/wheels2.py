import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from adafruit_servokit import ServoKit

# Motor channel definitions
FRONT_LEFT = 0
FRONT_RIGHT = 1
BACK_LEFT = 2
BACK_RIGHT = 3


class TwistToMotorDriver(Node):
    def __init__(self, wheel_dist):
        super().__init__("twist_to_motor_driver")

        # Initialize ServoKit for PCA9685
        self.kit = ServoKit(channels=16, frequency=50)
        for motor in (FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT):
            self.kit.continuous_servo[motor].set_pulse_width_range(450, 2700)

        self.wheel_dist = wheel_dist  # Distance between wheels

        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        self.get_logger().info("Twist to Motor Driver Node Initialized")

    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Calculate desired speeds for left and right motors
        speed_right = (angular_velocity * self.wheel_dist) / 2 + linear_velocity
        speed_left = linear_velocity * 2 - speed_right

        # Normalize speeds to [-1, 1] range
        max_speed = max(abs(speed_left), abs(speed_right), 1.0)
        speed_right /= max_speed
        speed_left /= max_speed

        # Set motor speeds
        self.set_motor_speed(FRONT_LEFT, speed_left)
        self.set_motor_speed(FRONT_RIGHT, speed_right)
        self.set_motor_speed(BACK_LEFT, speed_left)
        self.set_motor_speed(BACK_RIGHT, speed_right)

    def set_motor_speed(self, channel, speed, multiplier=(1.0, 1.0)):
        """
        Set motor speed.
        Speed range is -1 (full reverse) to 1 (full forward).
        """
        if speed < -0.1:
            adjusted_speed = speed * multiplier[0]
        elif speed < 0.1:
            adjusted_speed = 0
        else:
            adjusted_speed = speed * multiplier[1]
        self.kit.continuous_servo[channel].throttle = adjusted_speed

    def stop_all(self):
        """Stop all motors."""
        for channel in [FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT]:
            self.kit.continuous_servo[channel].throttle = 0

    def destroy(self):
        self.stop_all()


def main(args=None):
    rclpy.init(args=args)
    driver = TwistToMotorDriver(wheel_dist=0.5)

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
