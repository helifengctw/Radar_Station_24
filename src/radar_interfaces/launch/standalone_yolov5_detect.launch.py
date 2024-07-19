from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    Node_yolov5_detector = Node(
        package="yolov5_detect",
        executable="yolov5_detector_node",
        name="yolov5_detector",
        namespace="sensor_close",
        remappings=[('/sensor_close/raw/image', '/sensor_close/image_raw')],
        output="screen",
        emulate_tty=True,
        respawn=False
    )

    return LaunchDescription([Node_yolov5_detector])
