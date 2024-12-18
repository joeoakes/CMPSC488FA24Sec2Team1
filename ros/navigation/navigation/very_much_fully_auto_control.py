#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import TurretInstruction, WheelsInstruction
import curses


class MovementPublisher(Node):
    def __init__(self):
        super().__init__("movement_publisher")
        self.wheels_pub = self.create_publisher(WheelsInstruction, "/wheels_cmd", 1)
        self.turret_pub = self.create_publisher(TurretInstruction, "/turret_cmd", 1)

        curses.wrapper(self.loop)

    def loop(self, win):
        win.nodelay(True)
        key = ""
        win.clear()
        win.addstr("Detected key:")
        wheels = WheelsInstruction()
        wheels.speed = 1.0
        turret = TurretInstruction()
        while 1:
            try:
                key = win.getkey()

                wheels.direction = WheelsInstruction.STOP
                match str(key):
                    case " ":
                        turret.laser_duration = 0.5
                    case "r":
                        turret.zero_turret = True
                    case "KEY_LEFT":
                        turret.phi = 1
                    case "KEY_RIGHT":
                        turret.phi = -1
                    case "KEY_UP":
                        turret.theta = 1
                    case "KEY_DOWN":
                        turret.theta = -1
                    case "w":
                        wheels.direction = WheelsInstruction.FORWARD
                    case "s":
                        wheels.direction = WheelsInstruction.BACKWARD
                    case "d":
                        wheels.direction = WheelsInstruction.STRAFE_RIGHT
                    case "a":
                        wheels.direction = WheelsInstruction.STRAFE_LEFT
                    case "e":
                        wheels.direction = WheelsInstruction.TURN_RIGHT
                    case "q":
                        wheels.direction = WheelsInstruction.TURN_LEFT
                    case "p":
                        wheels.direction = WheelsInstruction.STOP

                self.wheels_pub.publish(wheels)
                self.turret_pub.publish(turret)

                win.clear()
                win.addstr("Detected key:")
                win.addstr(str(key))
            except KeyboardInterrupt:
                break
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    pub = MovementPublisher()
    rclpy.spin(pub)
