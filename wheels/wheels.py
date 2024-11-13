from adafruit_servokit import ServoKit
import time

# Initialize ServoKit instance for PCA9685 (with 16 channels)
kit = ServoKit(channels=20)

# Assign motor channels (adjust according to your wiring)
FRONT_LEFT = 0   # Channel for front-left motor
FRONT_RIGHT = 1  # Channel for front-right motor
BACK_LEFT = 2    # Channel for back-left motor
BACK_RIGHT = 3   # Channel for back-right motor

def set_motor_speed(channel, speed):
    """
    Set motor speed.
    Speed range is between -1 (full reverse) to 1 (full forward).
    """
    speed = max(-1, min(1, speed))  # Clamp speed to [-1, 1]
    pulse = int((speed + 1) * 500)  # Convert to 0-1000 pulse width range
    kit.continuous_servo[channel].throttle = speed

def stop_all():
    """Stop all motors."""
    for i in [FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT]:
        kit.continuous_servo[i].throttle = 0

# Define movement functions
def move_forward(speed=1):
    set_motor_speed(FRONT_LEFT, speed)
    set_motor_speed(FRONT_RIGHT, speed)
    set_motor_speed(BACK_LEFT, speed)
    set_motor_speed(BACK_RIGHT, speed)

def move_backward(speed=-1):
    set_motor_speed(FRONT_LEFT, speed)
    set_motor_speed(FRONT_RIGHT, speed)
    set_motor_speed(BACK_LEFT, speed)
    set_motor_speed(BACK_RIGHT, speed)

def move_left(speed=1):
    set_motor_speed(FRONT_LEFT, -speed)
    set_motor_speed(FRONT_RIGHT, speed)
    set_motor_speed(BACK_LEFT, speed)
    set_motor_speed(BACK_RIGHT, -speed)

def move_right(speed=1):
    set_motor_speed(FRONT_LEFT, speed)
    set_motor_speed(FRONT_RIGHT, -speed)
    set_motor_speed(BACK_LEFT, -speed)
    set_motor_speed(BACK_RIGHT, speed)

def rotate_clockwise(speed=1):
    set_motor_speed(FRONT_LEFT, speed)
    set_motor_speed(FRONT_RIGHT, -speed)
    set_motor_speed(BACK_LEFT, speed)
    set_motor_speed(BACK_RIGHT, -speed)

def rotate_counterclockwise(speed=1):
    set_motor_speed(FRONT_LEFT, -speed)
    set_motor_speed(FRONT_RIGHT, speed)
    set_motor_speed(BACK_LEFT, -speed)
    set_motor_speed(BACK_RIGHT, speed)

# Test movement
try:
    print("Moving forward")
    move_forward()
    time.sleep(2)

    print("Moving backward")
    move_backward()
    time.sleep(2)

    print("Moving left")
    move_left()
    time.sleep(2)

    print("Moving right")
    move_right()
    time.sleep(2)

    print("Spinning clockwise")
    rotate_clockwise()
    time.sleep(2)

    print("Spinning counterclockwise")
    rotate_counterclockwise()
    time.sleep(2)

finally:
    print("Stopping all motors")
    stop_all()

