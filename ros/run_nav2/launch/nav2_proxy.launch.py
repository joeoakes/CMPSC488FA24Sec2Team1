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
        ),
        Node(
            package = 'run_nav2',
            executable= 'goalpose',
            name = 'goalpose',
            output = "screen"
        ),
        Node(
            package = 'run_nav2',
            executable= 'odom_base_tf',
            name = 'odom_base_tf',
            output = 'screen'
        ),
        Node(
            package = 'run_nav2',
            executable= 'odom_test_data',
            name = 'odom_test_data',
            output = 'screen'
        )
    ])