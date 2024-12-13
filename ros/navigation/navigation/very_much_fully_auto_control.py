#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import curses


class MovementPublisher(Node):
    def __init__(self):
        super().__init__("movement_publisher")
        self.movement_pub = self.create_publisher(String, "/move_cmd", 10)

        # self.movement_pub.publish(")
        curses.wrapper(self.loop)

    def loop(self, win):
        win.nodelay(True)
        key = ""
        win.clear()
        win.addstr("Detected key:")
        movement = String()
        while 1:
            try:
                key = win.getkey()

                movement.data = "stop"
                match str(key):
                    case "w":
                        movement.data = "forward"
                    case "s":
                        movement.data = "backward"
                    case "d":
                        movement.data = "strafe_right"
                    case "a":
                        movement.data = "strafe_left"
                    case "e":
                        movement.data = "turn_right"
                    case "q":
                        movement.data = "turn_left"
                    case "p":
                        movement.data = "stop"
                self.movement_pub.publish(movement)

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
