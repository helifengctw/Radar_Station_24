from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    small_map_params = os.path.join(
        get_package_share_directory('radar_interfaces'),
        'config',
        'test_small_map_params.yaml'
    )

    Node_small_map = Node(
        package="small_map",
        executable="small_map",
        name="small_map",
        output="screen",
        emulate_tty=True,
        respawn=False,
        parameters=[small_map_params]
    )

    return LaunchDescription([
        Node_small_map
    ])
