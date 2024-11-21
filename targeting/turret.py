from typing import Tuple
from steppers.DRV8825 import DRV8825

FULL_STEPS_PER_ROT = 200
MICROSTEPS = 32

MICROSTEPS_PER_DEG = FULL_STEPS_PER_ROT * MICROSTEPS / 360
STEP_DELAY = 0.0001


class Turret:
    theta: float
    phi: float
    theta_motor: DRV8825
    phi_motor: DRV8825
    theta_limits: Tuple[float, float]
    phi_limits: Tuple[float, float]

    def __init__(
        self,
        theta_motor: DRV8825,
        phi_motor: DRV8825,
        theta_limits: Tuple[float, float],
        phi_limits: Tuple[float, float],
    ):
        self.theta = float(0)
        self.phi = float(0)
        self.theta_motor = theta_motor
        self.phi_motor = phi_motor
        self.theta_limits = theta_limits
        self.phi_limits = phi_limits

        self.theta_motor.set_micro_step("softward", "fullstep")
        self.phi_motor.set_micro_step("softward", "fullstep")

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

        theta_direction = "forward" if d_theta > 0 else "backward"
        phi_direction = "forward" if d_phi > 0 else "backward"

        self.theta_motor.turn_step(
            dir=theta_direction,
            steps=int(abs(d_theta * MICROSTEPS_PER_DEG)),
            stepdelay=STEP_DELAY,
        )
        self.phi_motor.turn_step(
            dir=phi_direction,
            steps=int(abs(d_phi * MICROSTEPS_PER_DEG)),
            stepdelay=STEP_DELAY,
        )

        self.stop()

        self.theta = new_theta
        self.phi = new_phi

    def move_to_zero(self):
        self.rotate(-self.theta, -self.phi)

    def reset_calibration(self):
        self.theta = 0
        self.phi = 0

    def stop(self):
        self.theta_motor.stop()
        self.phi_motor.stop()


def default_turret():
    motor1 = DRV8825(dir_pin=13, step_pin=19, enable_pin=12, mode_pins=(16, 17, 20))
    motor2 = DRV8825(dir_pin=24, step_pin=18, enable_pin=4, mode_pins=(21, 22, 27))

    theta_limits = (float("-inf"), float("inf"))
    phi_limits = (float("-inf"), float("inf"))  # TODO: Check these bounds

    return Turret(motor1, motor2, theta_limits, phi_limits)
