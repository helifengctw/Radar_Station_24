from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    Node_depth_sensor = Node(
        package="get_depth",
        executable="depth_sensor_node",
        name="depth_sensor",
        remappings=[('/sensor_far/livox/lidar', '/sensor_close/livox/lidar')],
        namespace="sensor_far",
        parameters=[
          {
            "camera_matrix": [2740.785753, 0.000000, 942.857551, 0.000000, 2730.148347, 595.336095, 0.000000, 0.000000, 1.000000],
            "distortion_coefficient": [-0.118461, 0.884219, -0.000027, -0.001337, 0.000000],
            "uni_matrix": [0.158509, -0.987162, 0.0196718, 0.13549, 0.122233, -0.000151194, -0.992501, -0.0109141, 0.979762, 0.159725, 0.12064, -0.100546],
            "length_of_cloud_queue": 10,
            "image_width": 1440, # 1280
            "image_height": 1080, # 1024,
            "camera_name": "sensor_far"
          },
        ],
        output="screen",
        emulate_tty=True,
        respawn=True
    )

    return LaunchDescription([Node_depth_sensor])
