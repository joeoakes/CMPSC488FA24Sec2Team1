#!/bin/bash

ros2 run move_wheels run_wheels &> /dev/null &
ros2 run navigation control
