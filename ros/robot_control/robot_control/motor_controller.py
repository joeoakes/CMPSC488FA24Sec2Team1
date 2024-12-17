from adafruit_servokit import ServoKit

# Initialize ServoKit instance for PCA9685 (with 16 channels)


class MotorController:
    # Assign motor channels (adjust according to your wiring)
    FRONT_LEFT = 0  # Channel for front-left motor
    FRONT_RIGHT = 1  # Channel for front-right motor
    BACK_LEFT = 2  # Channel for back-left motor
    BACK_RIGHT = 3  # Channel for back-right motor

    # Multipliers for each motor (can be adjusted as needed)
    # FRONT_LEFT_MULTIPLIER = 0.828
    # FRONT_RIGHT_MULTIPLIER = 1.0
    # BACK_LEFT_MULTIPLIER = 0.93
    # BACK_RIGHT_MULTIPLIER = 0.835

    # (reverse, forward)
    FRONT_LEFT_MULTIPLIER = (1.0, 1.0)
    FRONT_RIGHT_MULTIPLIER = (1.0, 1.0)
    BACK_LEFT_MULTIPLIER = (1.0, 1.0)
    BACK_RIGHT_MULTIPLIER = (1.0, 1.0)

    ALL_MOTORS = [FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT]

    def __init__(self):
        self.kit = ServoKit(channels=16, frequency=50)
        for motor in self.ALL_MOTORS:
            self.kit.continuous_servo[motor].set_pulse_width_range(450, 2700)

    def set_motor_speed(self, channel, speed, multiplier=(1.0, 1.0)):
        """
        Set motor speed.
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
        for i in [self.FRONT_LEFT, self.FRONT_RIGHT, self.BACK_LEFT, self.BACK_RIGHT]:
            self.kit.continuous_servo[i].throttle = 0

    # Methods to move individual wheels
    def move_front_left(self, speed):
        self.set_motor_speed(self.FRONT_LEFT, speed, self.FRONT_LEFT_MULTIPLIER)

    def move_front_right(self, speed):
        self.set_motor_speed(self.FRONT_RIGHT, speed, self.FRONT_RIGHT_MULTIPLIER)

    def move_back_left(self, speed):
        self.set_motor_speed(self.BACK_LEFT, speed, self.BACK_LEFT_MULTIPLIER)

    def move_back_right(self, speed):
        self.set_motor_speed(self.BACK_RIGHT, speed, self.BACK_RIGHT_MULTIPLIER)

    # Define movement functions
    def move_forward(self, speed):
        self.move_front_left(speed)
        self.move_front_right(-speed)
        self.move_back_left(speed)
        self.move_back_right(-speed)

    def move_backward(self, speed):
        self.move_front_left(-speed)
        self.move_front_right(speed)
        self.move_back_left(-speed)
        self.move_back_right(speed)

    def move_left(self, speed):
        self.move_front_left(-speed)
        self.move_front_right(-speed)
        self.move_back_left(speed)
        self.move_back_right(speed)

    def move_right(self, speed):
        self.move_front_left(speed)
        self.move_front_right(speed)
        self.move_back_left(-speed)
        self.move_back_right(-speed)

    def rotate_clockwise(self, speed):
        self.move_front_left(speed)
        self.move_front_right(speed)
        self.move_back_left(speed)
        self.move_back_right(speed)

    def rotate_counterclockwise(self, speed):
        self.move_front_left(-speed)
        self.move_front_right(-speed)
        self.move_back_left(-speed)
        self.move_back_right(-speed)


# Test movement for each wheel
if __name__ == "__main__":
    import time

    controller = MotorController()
    try:
        # Uncomment the desired wheel to run for 60 seconds

        # print("Running front-left wheel for 5 seconds")
        # controller.move_front_left(0.5)  # Set the desired speed (e.g., 1.0)
        # time.sleep(5)  # Run for 60 seconds

        # print("Running front-right wheel for 5 seconds")
        # controller.move_front_right(0.5)  # Set the desired speed (e.g., 1.0)
        # time.sleep(5)  # Run for 60 seconds
        #
        # print("Running back-left wheel for 5 seconds")
        # controller.move_back_left(0.5)  # Set the desired speed (e.g., 1.0)
        # time.sleep(5)  # Run for 60 seconds
        #
        # print("Running back-right wheel for 5 seconds")
        # controller.move_back_right(0.5)  # Set the desired speed (e.g., 1.0)
        # time.sleep(5)  # Run for 60 seconds
        controller.move_forward(speed=1)
        time.sleep(5)
        controller.move_backward(speed=1)
        time.sleep(5)
        controller.move_right(speed=1)
        time.sleep(5)
        controller.move_left(speed=1)
        time.sleep(5)
        controller.rotate_counterclockwise(speed=1)
        time.sleep(5)
        controller.rotate_clockwise(speed=1)
        time.sleep(5)
    finally:
        print("Stopping all motors")
        controller.stop_all()
