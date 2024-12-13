from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    name = "lazerbot_tf"
    package_share_dir = get_package_share_directory(name)

    path = os.path.join(
        package_share_dir,
        "src",
        "lazerbot_tf.urdf"
    )
    # Declare the path to URDF file as a launch argument
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value= path,
        description= "Generated path to urdf file"
    )
    
    # Create and return launch description with robot_state_publisher node
    return LaunchDescription([
        urdf_path_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('urdf_path')]),
                'use_sim_time': False
            }]
        )
    ])

    