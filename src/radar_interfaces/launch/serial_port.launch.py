from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    Node_serial_port = Node(
        package="serial_port",
        executable="serial_port_new",
        name="serial_port_new",
        output="screen",
        emulate_tty=True,
        parameters=[
          {
            "battle_color": "red"
          },
        ],
        respawn=True
    )

    return LaunchDescription([
        Node_serial_port
    ])
