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
        # remappings=[('/sensor_far/raw/image', '/sensor_far/image_raw')],
        parameters=[
          {
            "show_by_cv_or_msg": 0,
            "camera_name": "camera_close",
            "rgb_or_bayer": True,
            "light_gain": 2.0,
            "saturation_gain": 2.0,
            "image_width": 1920,
            "image_height": 1200, 
            "roi_x": 1919,
            "roi_y": 1,
            "last_diff_time_threshold": 400.0,
            "param_1": 250,
            "param_2": 150
          },
        ],
        output="screen",
        emulate_tty=True,
        respawn=False
    )

    return LaunchDescription([Node_yolov5_detector])
