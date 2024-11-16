from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("nav2_bringup"),"/launch","/bringup_launch.py"
            ])
        )
    ])