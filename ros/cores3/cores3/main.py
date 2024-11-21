#!/usr/bin/env bash

from geometry_msgs.msg import Quaternion
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import json

from std_msgs.msg import Header


class ImuPublisher(Node):
    def __init__(self):
        super().__init__("Cores3_Imu_Publisher")
        self.imu_pub = self.create_publisher(Imu, "/cores3/imu", 10)

        timer_period = 0.1  # seconds

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.serial = serial.Serial(
            port="/dev/ttyACM0",  # Find USB device using ls /dev
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
        )

    def timer_callback(self):
        if self.serial.in_waiting:
            line = self.serial.readline().strip()
            try:
                data = json.loads(line)
                imuData = Imu()
                imuData.header.stamp = self.get_clock().now()
                imuData.angular_velocity = data.gyro
                imuData.linear_acceleration = data.accel
                imuData.orientation = data.mag  # Quaternion()
                self.get_logger().info(data)
            except json.JSONDecodeError:
                self.get_logger().warn(f"Invalid JSON received: {line}")


def main(args=None):
    logger = rclpy.logging.get_logger("logger")
    rclpy.init(args=args)
    imu = ImuPublisher()
    try:
        logger.info("Started publishing Imu")
        rclpy.spin(imu)
    except (KeyboardInterrupt, ExternalShutdownException):
        logger.info("Shutting down gracefully")
        pass

    imu.serial.close()
    imu.destroy_node()
