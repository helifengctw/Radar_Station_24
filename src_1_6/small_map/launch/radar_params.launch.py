from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('small_map'),
        'config',
        'radar_params.yaml'
    	)

    return LaunchDescription([
        Node(
            package="small_map",
            executable="small_map",
            name="small_map",
            output="screen",
            emulate_tty=True,
            parameters=[config]
        )
    ])
