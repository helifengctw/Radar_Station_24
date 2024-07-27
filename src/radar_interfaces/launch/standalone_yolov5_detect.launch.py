from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    Node_yolov5_detector = Node(
        package="yolov5_detect",
        executable="yolov5_detector_node",
        name="yolov5_detector",
        namespace="sensor_far",
        remappings=[('/sensor_far/raw/image', '/sensor_far/image_raw')],
        parameters=[
          {
            "show_by_cv_or_msg": 0,
            "camera_name": "sensor_far",
            "rgb_or_bayer": False,
            "light_gain": 2.0,
            "saturation_gain": 2.0
          },
        ],
        output="screen",
        emulate_tty=True,
        respawn=False
    )

    return LaunchDescription([Node_yolov5_detector])
