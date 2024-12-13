colcon build
. install/setup.bash
ros2 launch lazerbot_tf tfpublish.launch.py >/dev/null 2>&1  &
sleep 2
ros2 launch run_slam slam.launch.py >/dev/null 2>&1 &
sleep 2
ros2 run wheels run_wheels &
ros2 launch run_nav2 nav2_proxy.launch.py 

