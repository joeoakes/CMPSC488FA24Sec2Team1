from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("nav2_bringup"), "params", "nav2_params.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="run_nav2",
                executable="goalpose",
                name="goalpose",
                output="screen",
            ),
            Node(
                package="run_nav2",
                executable="rtabodom",
                name="rtabodom",
                output="screen",
            ),
            Node(
                package="run_nav2",
                executable="maprelay",
                name="maprelay",
                output="screen",
            ),
            Node(
                package="run_nav2",
                executable="odom_base_tf",
                name="odom_base_tf",
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("nav2_bringup"),
                        "launch",
                        "bringup_launch.py",
                    )
                ),
                launch_arguments={
                    "global_costmap.global_costmap.allow_unknown": "true",
                    "global_costmap.global_costmap.track_unknown_space": "true",
                    "local_costmap.local_costmap.allow_unknown": "true",
                    "local_costmap.local_costmap.track_unknown_space": "true",
                    "planner_server.allow_unknown": "true",
                    "global_costmap.global_costmap.obstacle_layer.enabled": "false",
                    "local_costmap.local_costmap.obstacle_layer.enabled": "false",
                    "use_sim_time": "false",
                    "params_file": params_file,
                    "map_server.autostart": "false",
                    "map_server.enabled": "false",
                }.items(),
            ),
        ]
    )
