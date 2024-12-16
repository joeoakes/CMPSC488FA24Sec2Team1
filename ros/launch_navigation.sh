#!/bin/bash

ros2 run move_wheels run_wheels &> /dev/null &
ros2 run laser_control laser_control_node &> /dev/null &
ros2 run navigation control
