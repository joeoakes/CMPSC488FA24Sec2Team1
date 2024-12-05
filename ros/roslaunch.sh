colcon build
. install/setup.bash
ros2 launch lazerbot_tf tfpublish.launch.py
ros2 launch run_nav2 nav2_proxy.launch.py
ros2 launch run_slam slam.launch.py
