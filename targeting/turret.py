from time import sleep, thread_time

# test
from typing import Tuple

from adafruit_servokit import ServoKit


class Turret:
    theta: float
    phi: float
    _servo_kit: ServoKit
    theta_channel: int
    phi_channel: int
    theta_limits: Tuple[float, float]
    phi_limits: Tuple[float, float]

    def __init__(
        self,
        theta_channel: int,
        phi_channel: int,
        theta_limits: Tuple[float, float],
        phi_limits: Tuple[float, float],
        theta_zero=0,
        phi_zero=90,
    ):
        self.servo_kit = ServoKit(channels=16)
        self.theta_channel = theta_channel
        self.phi_channel = phi_channel
        self.theta_limits = theta_limits
        self.phi_limits = phi_limits
        self.theta = float(theta_zero)
        self.phi = float(phi_zero)
        self.theta_zero = theta_zero
        self.phi_zero = phi_zero

        self.servo_kit.servo[self.theta_channel].set_pulse_width_range(500, 2500)
        self.servo_kit.servo[self.phi_channel].set_pulse_width_range(500, 2500)
        self.slow_zero()

    def rotate(self, d_theta: float, d_phi: float):
        """
        Takes 2 angles in degrees to move the turret.
        Checks if new angle is within bounds.
        If new angle is within bounds, moves motor that amount, then updates current angles.
        Params:
            d_theta: Left-right change of angle in degrees. Positive values indicate counter-clockwise rotation
            d_phi: Up-down change of angle in degrees. Positive values indicate down (counter-clockwise)
        """
        new_theta = self.theta + d_theta
        new_phi = self.phi + d_phi

        if new_theta < self.theta_limits[0] or new_theta > self.theta_limits[1]:
            raise ValueError("d_theta will take the theta angle outside the limits")

        if new_phi < self.phi_limits[0] or new_phi > self.phi_limits[1]:
            raise ValueError("d_phi will take the phi angle outside the limits")

        self.servo_kit.servo[self.theta_channel].angle = new_theta
        self.servo_kit.servo[self.phi_channel].angle = new_phi

        self.theta = new_theta
        self.phi = new_phi

    def move_to_zero(self):
        self.servo_kit.servo[self.theta_channel].angle = self.theta_zero
        self.servo_kit.servo[self.phi_channel].angle = self.phi_zero
        self.theta = self.theta_zero
        self.phi = self.phi_zero

    def slow_zero(self, delay=0.1):
        theta_step = 1 if self.theta_zero > self.theta else -1
        phi_step = 1 if self.phi_zero > self.phi else -1

        for theta in range(int(self.theta), self.theta_zero, theta_step):
            self.servo_kit.servo[self.theta_channel].angle = theta
            sleep(delay)

        for phi in range(int(self.phi), self.phi_zero, phi_step):
            self.servo_kit.servo[self.phi_channel].angle = phi
            sleep(delay)

        # just in case
        self.move_to_zero()

    def reset_calibration(self):
        self.theta = 0
        self.phi = 0


def default_turret():
    theta_limits = (float("-inf"), float("inf"))
    phi_limits = (float("-inf"), float("inf"))  # TODO: Check these bounds

    return Turret(8, 9, theta_limits, phi_limits)


if __name__ == "__main__":
    turret = default_turret()
    for i in range(0, 12):
        print(i)
        turret.rotate(i, i)
        sleep(0.5)
    turret.move_to_zero()
