#!/usr/bin/env bash

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import json


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
        self.get_logger().info("Cores3 Imu Publisher started")

    def timer_callback(self):
        if self.serial.in_waiting:
            line = self.serial.readline().strip()
            try:
                data = json.loads(line)
                imuData = Imu()
                imuData.header.frame_id = "cores3_imu"
                imuData.header.stamp = self.get_clock().now()

                imuData.angular_velocity.z = data.gyro.z
                imuData.angular_velocity.y = data.gyro.y
                imuData.angular_velocity.z = data.gyro.z

                imuData.linear_acceleration.z = data.accel.z
                imuData.linear_acceleration.y = data.accel.y
                imuData.linear_acceleration.z = data.accel.z

                imuData.orientation.w = 1
                imuData.orientation.z = data.mag.z
                imuData.orientation.y = data.mag.y
                imuData.orientation.z = data.mag.z
                self.imu_pub.publish(imuData)
            except json.JSONDecodeError:
                self.get_logger().warn(f"Invalid JSON received: {line}")


def main(args=None):
    rclpy.init(args=args)
    imu = ImuPublisher()
    try:
        rclpy.spin(imu)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    imu.serial.close()
    imu.destroy_node()
