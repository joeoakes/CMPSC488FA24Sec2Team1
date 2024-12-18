from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Node(
            #     package="log_to_mongodb", executable="start_log", name="log_to_mongodb"
            # ),
            Node(package="kinect_ros_jazzy", executable="publish", name="kinect"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("run_slam"),
                                "rtabmap.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "args": "--delete_db_on_start",
                    "rgb_topic": "/kinect2/rgb/image",
                    "depth_topic": "/kinect2/depth",
                    "camera_info_topic": "/kinect2/rgb/camera_info",
                    "frame_id": "kinect_link",
                    "approx_sync": "true",
                    "rtabmap_viz": "false",
                    # "rviz": "true",
                }.items(),
            ),
        ]
    )
