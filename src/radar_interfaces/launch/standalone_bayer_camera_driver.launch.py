from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    camera_params = os.path.join(
            get_package_share_directory('radar_interfaces'),
            'config',
            'camera_driver_params.yaml'
        )
    Node_bayer_camera_driver = Node(
        package="bayer_camera_driver",
        executable="hikvision_driver_node",
        namespace='sensor_close',
        name='bayer_camera_driver',
        parameters=[
          {
            "camera_name": 'camera_close',
            "camera_height": 1200,
            "camera_width": 1920,
            "camera_exp": 3500.0,
            "camera_gain": 23.98,
            "camera_auto_exp": 0,
            "camera_auto_maxexp": 4500.0,
            "camera_auto_minexp": 100.0,
            "camera_auto_gain": 0,
            "camera_auto_maxgain": 17.0,
            "camera_auto_mingain": 0.0,
            "camera_auto_whitebalance": 1,
          }
        ],
        output="screen",
        emulate_tty=True,
      )

    return LaunchDescription([Node_bayer_camera_driver])

